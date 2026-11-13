"""线路 B 独立状态机。

    LYING -> GETUP -> STAND_HOLD -> LIEDOWN -> LYING

本模块不定义、导入或接受 FOLDED / UNFOLD / FOLD，避免线路 A 的收储状态进入本控制器。
实验性的起身→行走交接必须显式传入配置。缺省只允许 MuJoCo；真机只有独立的
一次性零速验收程序或显式开启的人工行走/转向入口可以传入专用 capability 开关。
缺省状态图仍保持事故锁定，人工行走与舞蹈能力彼此独立。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import time
from pathlib import Path

import numpy as np

from apps.teleop.command import BACK, FWD, TURN, fill_goal, ramp
from apps.teleop.gamepad import Pad
from apps.teleop.lying_pose import is_folded_prone
from apps.teleop.voice import Silent
from runtime import Backend, MuJoCoBackend, Package, Session, SkillModel, TeleGoal
from runtime.body_ref import BodyRef
from runtime.primebot_io import vendor_mc_running

LYING, GETUP, STAND_HOLD, WALK_HANDOFF, WALK_SETTLE, WALK, STRAIGHTEN, LIEDOWN, DANCE, DAMPED = (
    "LYING", "GETUP", "STAND_HOLD", "WALK_HANDOFF", "WALK_SETTLE", "WALK", "STRAIGHTEN", "LIEDOWN", "DANCE", "DAMPED"
)
FALL_UPRIGHT = 0.5
LYING_UPRIGHT = 0.5
START_POSE_JOINT_MAX = 0.7
SETTLE_GYRO = 0.3
MANUAL_WALK_MSG = "线路 B：按住前进/后退/转向才移动；松手归零，可正常停止或趴下"
FULL_WALK_MSG = "线路 B：原仓库真机速度前后行走和转向；可跳舞、正常停止或趴下"


@dataclass(frozen=True)
class HandoffConfig:
    duration_s: float = 10.0
    settle_s: float = 3.0
    # 只在交接开始前检查两个策略是否大体兼容。过渡中两策略最新缓存目标的差值仅记录，
    # 因为它不是实际下发跳变；下发跳变由 target_step_max 单独限制。
    initial_gap_max: float = 0.50
    target_step_max: float = 0.01
    upright_min: float = 0.85
    gyro_max: float = 0.50
    tracking_max: float = 0.35

    def validate(self) -> None:
        if any(float(v) <= 0 for v in asdict(self).values()):
            raise ValueError("交接配置全部参数必须大于 0")


@dataclass
class HandoffMetrics:
    initial_policy_gap: float = 0.0
    max_policy_gap: float = 0.0
    max_raw_target_step: float = 0.0
    max_applied_target_step: float = 0.0
    max_tracking_error: float = 0.0
    min_upright: float = 1.0
    max_gyro: float = 0.0
    final_base_residual: float = float("inf")
    failure: str = ""

    def as_dict(self) -> dict:
        out = asdict(self)
        return {
            k: (round(float(v), 6) if math.isfinite(v) else None) if isinstance(v, float) else v
            for k, v in out.items()
        }


def smoothstep01(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


class LineBMachine:
    def __init__(self, base: Package, skill: Package, robot: Backend, getup_ref: Path, liedown_ref: Path,
                 dance: Package, dance_ref: Path, speed_scale: float,
                 blend_s: float = 0.4,
                 settle_s: float = 1.5, voice: Silent | None = None,
                 experimental_handoff: HandoffConfig | None = None,
                 allow_real_handoff_validation: bool = False,
                 allow_real_manual_drive: bool = False,
                 dance_enabled: bool = False):
        compatible = (
            skill.joint_names == base.joint_names
            and np.allclose(skill.default_joint_pos, base.default_joint_pos)
            and np.allclose(skill.action_scale, base.action_scale)
            and np.isclose(skill.control_dt, base.control_dt)
        )
        if not compatible:
            raise ValueError("起身技能包与基础模型不兼容")
        dance_compatible = (
            dance.joint_names == base.joint_names
            and np.allclose(dance.default_joint_pos, base.default_joint_pos)
            and np.allclose(dance.action_scale, base.action_scale)
            and np.isclose(dance.control_dt, base.control_dt)
        )
        if not dance_compatible:
            raise ValueError("舞蹈技能包与基础模型不兼容")
        self.base, self.robot = base, robot
        self.sim = isinstance(robot, MuJoCoBackend)
        allow_real_handoff = allow_real_handoff_validation or allow_real_manual_drive
        if allow_real_handoff and experimental_handoff is None:
            raise ValueError("真机交接 capability 必须与交接配置同时使用")
        if allow_real_handoff_validation and allow_real_manual_drive:
            raise ValueError("一次性零速验收与人工行走 capability 不能同时启用")
        if experimental_handoff is not None and not self.sim and not allow_real_handoff:
            raise ValueError("实验交接在普通入口只允许 MuJoCoBackend，禁止真机后端")
        if experimental_handoff is not None:
            experimental_handoff.validate()
        self.s = Session(base, robot)
        self.goal = TeleGoal(base)
        self.skill = SkillModel(skill)
        self.getup = BodyRef(skill, getup_ref)
        self.liedown = BodyRef(skill, liedown_ref)
        self.dance_skill = SkillModel(dance)
        self.dance_ref = BodyRef(dance, dance_ref)
        self.pd_base = (base.kp, base.kd)
        self.pd_skill = (skill.kp, skill.kd)
        self.pd_dance = (dance.kp, dance.kd)
        self.dt = base.control_dt
        self.speed = float(speed_scale)
        if self.speed < 0.0:
            raise ValueError("原仓库统一速度比例不能为负")
        self.dance_enabled = bool(dance_enabled)
        self.walk_msg = FULL_WALK_MSG if self.dance_enabled else MANUAL_WALK_MSG
        self.blend_n = max(1, int(round(blend_s / self.dt)))
        self.settle_s = float(settle_s)
        self.voice = voice or Silent()
        self.mode, self.msg = LYING, "线路 B：等待标准平趴姿起身"
        self.pose_check_result = {
            "state": "not_run", "passed": None, "upright": None,
            "max_joint_error": None, "worst_joint": "", "folded": None,
            "reason": "尚未检查", "checked_count": 0,
        }
        self.cmd = np.zeros(6)
        self.i = 0
        self.t_mode = 0.0
        self.fade = None
        self.after_straighten: str | None = None
        self.target = self.kp = self.kd = None
        self.handoff = experimental_handoff
        self.handoff_label = "仅仿真实验" if self.sim else \
            "线路 B 人工行走和转向" if allow_real_manual_drive else "一次性真机零速验收"
        self.handoff_metrics = HandoffMetrics()
        self.handoff_n = max(1, int(round(experimental_handoff.duration_s / self.dt))) \
            if experimental_handoff is not None else 0
        self.handoff_settle_n = max(1, int(round(experimental_handoff.settle_s / self.dt))) \
            if experimental_handoff is not None else 0
        self.handoff_i = self.handoff_settle_i = 0
        self.handoff_anchor_target: np.ndarray | None = None
        self.handoff_skill_target: np.ndarray | None = None
        self.handoff_base_target: np.ndarray | None = None
        self.handoff_prev_target: np.ndarray | None = None
        if self.sim:
            self.place_lying()

    def _enter(self, mode: str, msg: str, say: str | None = None) -> None:
        self.mode, self.msg, self.t_mode = mode, msg, 0.0
        print(f"[line-b] {mode:10s} {msg}", flush=True)
        if say:
            self.voice.say(say)

    def place_lying(self) -> None:
        r = self.getup
        self.robot.set_state(r.root_pos[0], r.root_quat[0], r.joint_pos[0])
        self._enter(LYING, "线路 B：标准平趴姿，可起身")

    def place_standing(self) -> None:
        self.robot.reset()
        self.s.begin()
        self.cmd[:], self.fade = 0.0, None
        self._enter(WALK, self.walk_msg)

    def damp(self, why: str) -> None:
        self.robot.damp()
        self.fade = None
        self.cmd[:] = 0.0
        self._enter(DAMPED, f"{why}，已切阻尼；只有重新摆到标准平趴姿才允许再次起身")

    def _evaluate_pose(self) -> dict:
        """只读检查线路 B 平趴首姿；不调用 reset / step / set_pd。"""
        sensed = self.robot.sense()
        upright = float(self.robot.upright())
        joint_pos = np.asarray(sensed.joint_pos, dtype=np.float64)
        dq = joint_pos - self.getup.joint_pos[0]
        worst = int(np.argmax(np.abs(dq)))
        error = float(abs(dq[worst]))
        joint = self.base.joint_names[worst]
        folded = bool(is_folded_prone(self.base.joint_names, joint_pos))
        reason = ""
        if upright > LYING_UPRIGHT:
            reason = f"upright {upright:+.2f} > {LYING_UPRIGHT:.2f}：不是俯卧"
        elif folded:
            reason = "检测到深度折叠：线路 B 禁止起身，请退出并改用线路 A"
        elif error > START_POSE_JOINT_MAX:
            reason = (f"不接近标准平趴首姿：最大关节差 {error:.2f} > "
                      f"{START_POSE_JOINT_MAX:.2f} rad（{joint}）")
        result = {
            "state": "fail" if reason else "pass",
            "passed": not reason,
            "upright": upright,
            "max_joint_error": error,
            "worst_joint": joint,
            "folded": folded,
            "reason": reason or "标准平趴姿检查通过",
            "checked_count": int(getattr(self, "pose_check_result", {}).get("checked_count", 0)) + 1,
        }
        self.pose_check_result = result
        return result

    def _check_pose(self) -> None:
        result = self._evaluate_pose()
        if result["passed"]:
            self.msg = (f"摆姿检查通过：upright {result['upright']:+.2f}，最大关节差 "
                        f"{result['max_joint_error']:.2f} rad（{result['worst_joint']}）；未下发动作")
        else:
            self.msg = f"摆姿检查未通过：{result['reason']}；未下发动作"
        print(f"[line-b-pose] {self.msg}", flush=True)

    def _start_getup(self) -> None:
        if not self.sim and vendor_mc_running():
            self.msg = "厂商运控 mc 正在运行，拒绝线路 B 起身"
            return
        check = self._evaluate_pose()
        if not check["passed"]:
            self.msg = f"摆姿检查未通过：{check['reason']}，拒绝起身"
            return
        self.robot.set_pd(*self.pd_skill)
        if self.sim:
            self.place_lying()
        else:
            self.robot.reset(self.getup.joint_pos[0])
        self.s.begin()
        self.getup.start(self.robot.torso_quat())
        self.i = 0
        self._enter(GETUP, f"线路 B 起身 {(self.getup.T - 1) * self.dt:.1f} s")

    def _start_liedown(self, from_skill_hold: bool = False) -> None:
        self.liedown.start(self.robot.torso_quat())
        self.i = 0
        self.fade = (self._ref_policy(self.skill, self.getup, self.getup.T - 1), self.pd_skill, self.blend_n) \
            if from_skill_hold else (self._tele_policy, self.pd_base, self.blend_n)
        self._enter(LIEDOWN, f"趴下 {(self.liedown.T - 1) * self.dt:.1f} s；完成后自动阻尼")

    def _start_dance(self) -> None:
        if not self.dance_enabled:
            self.msg = "事故锁定：舞蹈尚未放行"
            return
        self.dance_ref.start(self.robot.torso_quat())
        self.i = 0
        self.fade = (self._tele_policy, self.pd_base, self.blend_n)
        self._enter(DANCE, f"舞蹈 {(self.dance_ref.T - 1) * self.dt:.1f} s")

    def _ref_policy(self, model: SkillModel, ref: BodyRef, i: int):
        return lambda obs: model.act(obs, ref.goal(i, self.robot.torso_quat()))

    def _tele_policy(self, obs: np.ndarray):
        fill_goal(self.goal, self.cmd)
        return self.s.model.act_tele(obs, self.goal)

    def _zero_speed_base_policy(self, obs: np.ndarray):
        """基础策略的零速调用；交接每个控制帧最多运行一次策略推理。

        交接段由 _handoff_step 在奇偶帧交替运行起身/基础策略，
        避免真机 CPU 在同一控制帧串行执行两次 ONNX 推理。
        """
        self.cmd[:] = 0.0
        return self._tele_policy(obs)

    def _apply_handoff(self, target: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
        action = (target - self.s.q0) / self.base.action_scale
        self.robot.set_pd(kp, kd)
        self.s.apply(action, target)
        self.target, self.kp, self.kd = target, kp, kd

    def _fail_handoff(self, reason: str) -> None:
        self.handoff_metrics.failure = reason
        self.damp(f"零速交接门禁失败：{reason}")

    def _measure_handoff(self, target: np.ndarray) -> bool:
        sensed = self.robot.sense()
        upright = self.robot.upright()
        gyro = float(np.linalg.norm(sensed.base_ang_vel))
        tracking = float(np.abs(target - sensed.joint_pos).max())
        m, cfg = self.handoff_metrics, self.handoff
        m.min_upright = min(m.min_upright, upright)
        m.max_gyro = max(m.max_gyro, gyro)
        m.max_tracking_error = max(m.max_tracking_error, tracking)
        if upright < cfg.upright_min:
            self._fail_handoff(f"upright {upright:.3f} < {cfg.upright_min:.3f}")
            return False
        if gyro > cfg.gyro_max:
            self._fail_handoff(f"gyro {gyro:.3f} > {cfg.gyro_max:.3f} rad/s")
            return False
        if tracking > cfg.tracking_max:
            self._fail_handoff(f"tracking {tracking:.3f} > {cfg.tracking_max:.3f} rad")
            return False
        return True

    def _start_handoff(self) -> None:
        if self.handoff is None:
            self.msg = "事故锁定：起身到行走交接未启用"
            return
        obs = self.s.obs()
        # 上一帧已实际下发的起身末目标；交接中不再重复运行起身策略。
        anchor = self.s.q0 + self.base.action_scale * np.asarray(self.s.last_action, dtype=np.float64)
        _, base_target = self._zero_speed_base_policy(obs)
        gap = float(np.abs(base_target - anchor).max())
        self.handoff_metrics = HandoffMetrics(initial_policy_gap=gap, max_policy_gap=gap)
        if gap > self.handoff.initial_gap_max:
            self._fail_handoff(f"initial policy gap {gap:.3f} > {self.handoff.initial_gap_max:.3f} rad")
            return
        self.handoff_i = 0
        self.handoff_anchor_target = np.asarray(anchor, dtype=np.float64)
        self.handoff_skill_target = self.handoff_anchor_target.copy()
        self.handoff_base_target = np.asarray(base_target, dtype=np.float64)
        self.handoff_prev_target = self.handoff_anchor_target.copy()
        self.fade = None
        self._enter(WALK_HANDOFF, f"{self.handoff_label}：慢交接 {self.handoff.duration_s:.1f} s；零速度")

    def _handoff_step(self) -> None:
        obs = self.s.obs()
        # 每个 50 Hz 控制帧最多一次 ONNX 推理；两策略各以 25 Hz 更新缓存。
        if self.handoff_i % 2 == 0:
            _, target = self._ref_policy(self.skill, self.getup, self.getup.T - 1)(obs)
            self.handoff_skill_target = np.asarray(target, dtype=np.float64)
        else:
            _, target = self._zero_speed_base_policy(obs)
            self.handoff_base_target = np.asarray(target, dtype=np.float64)
        skill_target, base_target = self.handoff_skill_target, self.handoff_base_target
        m, cfg = self.handoff_metrics, self.handoff
        m.max_policy_gap = max(m.max_policy_gap, float(np.abs(base_target - skill_target).max()))
        w = smoothstep01((self.handoff_i + 1) / self.handoff_n)
        raw = (1.0 - w) * skill_target + w * base_target
        previous = self.handoff_prev_target
        raw_step = float(np.abs(raw - previous).max())
        target = previous + np.clip(raw - previous, -cfg.target_step_max, cfg.target_step_max)
        applied_step = float(np.abs(target - previous).max())
        m.max_raw_target_step = max(m.max_raw_target_step, raw_step)
        m.max_applied_target_step = max(m.max_applied_target_step, applied_step)
        kp = (1.0 - w) * self.pd_skill[0] + w * self.pd_base[0]
        kd = (1.0 - w) * self.pd_skill[1] + w * self.pd_base[1]
        self._apply_handoff(target, kp, kd)
        self.handoff_prev_target = target
        if not self._measure_handoff(target):
            return
        self.handoff_i += 1
        if self.handoff_i >= self.handoff_n:
            m.final_base_residual = float(np.abs(target - base_target).max())
            self.handoff_settle_i = 0
            self._enter(WALK_SETTLE, f"{self.handoff_label}：基础策略零速观察 {cfg.settle_s:.1f} s")

    def _handoff_settle_step(self) -> None:
        obs = self.s.obs()
        _, raw = self._tele_policy(obs)
        previous, cfg, m = self.handoff_prev_target, self.handoff, self.handoff_metrics
        raw_step = float(np.abs(raw - previous).max())
        target = previous + np.clip(raw - previous, -cfg.target_step_max, cfg.target_step_max)
        applied_step = float(np.abs(target - previous).max())
        m.max_raw_target_step = max(m.max_raw_target_step, raw_step)
        m.max_applied_target_step = max(m.max_applied_target_step, applied_step)
        self._apply_handoff(target, self.pd_base[0], self.pd_base[1])
        self.handoff_prev_target = target
        m.final_base_residual = float(np.abs(target - raw).max())
        if not self._measure_handoff(target):
            return
        self.handoff_settle_i += 1
        if self.handoff_settle_i >= self.handoff_settle_n:
            if m.final_base_residual > cfg.target_step_max:
                self._fail_handoff(f"base residual {m.final_base_residual:.3f} > {cfg.target_step_max:.3f} rad")
                return
            self.cmd[:] = 0.0
            self._enter(WALK, f"{self.handoff_label}通过：基础策略零速站立；{self.walk_msg}")

    def _act(self, policy, pd: tuple[np.ndarray, np.ndarray]) -> None:
        obs = self.s.obs()
        action, target = policy(obs)
        kp, kd = pd
        if self.fade is not None:
            old, (kp_old, kd_old), left = self.fade
            _, q_old = old(obs)
            w = 1.0 - (left - 1) / self.blend_n
            target = (1.0 - w) * q_old + w * target
            action = (target - self.s.q0) / self.base.action_scale
            kp, kd = (1.0 - w) * kp_old + w * kp, (1.0 - w) * kd_old + w * kd
            self.fade = (old, (kp_old, kd_old), left - 1) if left > 1 else None
        self.robot.set_pd(kp, kd)
        self.s.apply(action, target)
        self.target, self.kp, self.kd = target, kp, kd

    def _idle(self) -> None:
        if self.sim:
            self.robot.damp()
            self.robot.step(self.robot.sense().joint_pos)
        else:
            time.sleep(self.dt)

    def tick(self, pad: Pad, edges: set[str]) -> None:
        self.target = self.kp = self.kd = None
        try:
            self._tick(pad, edges)
        except RuntimeError as e:
            self.damp(f"后端故障：{e}")
        self.t_mode += self.dt

    def _tick(self, pad: Pad, edges: set[str]) -> None:
        if "DAMP" in edges and self.mode not in (LYING, DAMPED):
            return self.damp("急停")
        if self.mode in (LYING, DAMPED):
            if "POSE_CHECK" in edges:
                return self._check_pose()
            if "GETUP" in edges or (pad.l2 and "A" in edges):
                return self._start_getup()
            return self._idle()

        if self.mode == STAND_HOLD:
            if "WALK" in edges:
                return self._start_handoff()
            if "LIEDOWN" in edges or (pad.l2 and "B" in edges):
                return self._start_liedown(from_skill_hold=True)
            self._act(self._ref_policy(self.skill, self.getup, self.getup.T - 1), self.pd_skill)
            if self.robot.upright() < FALL_UPRIGHT:
                return self.damp("站立保持中倾倒")
            return

        if self.mode == WALK_HANDOFF:
            return self._handoff_step()

        if self.mode == WALK_SETTLE:
            return self._handoff_settle_step()

        if self.mode in (GETUP, LIEDOWN):
            ref, model, pd = (self.getup, self.skill, self.pd_skill) if self.mode == GETUP \
                else (self.liedown, self.skill, self.pd_skill)
            self._act(self._ref_policy(model, ref, self.i), pd)
            self.i += 1
            if self.i < ref.T - 1:
                return
            if self.mode == LIEDOWN:
                self.robot.damp()
                self.cmd[:] = 0.0
                return self._enter(LYING, "趴下完成，已切阻尼")
            self.cmd[:] = 0.0
            self.fade = None
            self.i = ref.T - 1
            next_step = ("可点击进入人工行走和转向；进入 WALK 后可跳舞"
                         if self.dance_enabled else
                         "可点击进入人工行走和转向，舞蹈仍锁定") \
                if self.handoff is not None else "行走和舞蹈事故锁定，可点击趴下"
            return self._enter(STAND_HOLD, f"起身完成，保持起身策略末帧；{next_step}")

        if self.mode == DANCE:
            self._act(self._ref_policy(self.dance_skill, self.dance_ref, self.i), self.pd_dance)
            self.i += 1
            if self.robot.upright() < FALL_UPRIGHT:
                return self.damp("舞蹈中倾倒")
            if self.i < self.dance_ref.T - 1:
                return
            self.cmd[:] = 0.0
            self.fade = (self._ref_policy(self.dance_skill, self.dance_ref, self.dance_ref.T - 1),
                         self.pd_dance, self.blend_n)
            return self._enter(WALK, f"舞蹈完成，已回到站立；{self.walk_msg}")

        if self.mode == WALK:
            if "LIEDOWN" in edges or (pad.l2 and "B" in edges):
                self.after_straighten = "liedown"
                return self._enter(STRAIGHTEN, "速度归零并站稳，准备趴下")
            if self.dance_enabled and ("DANCE" in edges or (pad.l2 and "X" in edges)):
                self.after_straighten = "dance"
                return self._enter(STRAIGHTEN, "速度归零并站稳，准备跳舞")
            if "DANCE" in edges or (pad.l2 and "X" in edges):
                self.msg = "事故锁定：舞蹈尚未放行"
            # 与 apps.teleop.machine.Teleop 共用同一映射：前进、后退、转向只用
            # 一个 speed_scale，再经 apps.teleop.command.ramp 做原仓库加速度限幅。
            vx = self.speed * (FWD if pad.ly > 0 else BACK) * pad.ly
            wz = self.speed * TURN * pad.rx
            target = np.array([vx, 0.0, wz, 0.0, 0.0, 0.0])
        else:
            target = np.zeros(6)
        self.cmd = ramp(self.cmd, target, self.dt)
        self._act(self._tele_policy, self.pd_base)
        if self.robot.upright() < FALL_UPRIGHT:
            return self.damp("行走中倾倒")
        if self.mode == STRAIGHTEN and self.t_mode >= self.settle_s and np.abs(self.cmd).max() < 1e-3 \
                and np.linalg.norm(self.robot.sense().base_ang_vel) < SETTLE_GYRO:
            action, self.after_straighten = self.after_straighten, None
            self._start_dance() if action == "dance" else self._start_liedown()

    def status(self) -> list[tuple[str, str]]:
        rows = [("route", "B"), ("mode", self.mode), ("", self.msg)]
        if self.mode in (WALK, STRAIGHTEN):
            rows += [("vx", f"{self.cmd[0]:+.2f}"), ("wz", f"{self.cmd[2]:+.2f}")]
        elif self.mode in (GETUP, STAND_HOLD, WALK_HANDOFF, WALK_SETTLE, LIEDOWN, DANCE):
            if self.mode in (WALK_HANDOFF, WALK_SETTLE):
                total = self.handoff.duration_s if self.mode == WALK_HANDOFF else self.handoff.settle_s
                frame = self.handoff_i if self.mode == WALK_HANDOFF else self.handoff_settle_i
                rows.append(("t", f"{frame * self.dt:.1f} / {total:.1f} s"))
                return rows
            ref = self.dance_ref if self.mode == DANCE else self.getup if self.mode == GETUP else self.liedown
            if self.mode == STAND_HOLD:
                ref = self.getup
            rows.append(("t", f"{self.i * self.dt:.1f} / {(ref.T - 1) * self.dt:.1f} s"))
        return rows
