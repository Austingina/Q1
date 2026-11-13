"""手柄遥操作的状态机，每个控制周期 ``tick`` 一次（仿真和真机同一套）：

    FOLDED ──起身──▶ UNFOLD（深度折叠→平趴）──▶ GETUP（平趴→站立）──▶ WALK
       ▲                                                          │ 趴下 / 折叠 / 舞蹈
       └── FOLD（平趴→深度折叠）◀── LIEDOWN ◀── STRAIGHTEN       └──▶ DANCE ──▶ WALK

    普通“趴下”在 LIEDOWN 完成后切阻尼并停在 LYING；“折叠 / 准备关机”才会继续执行 FOLD。
    FOLD 完成后保持最后一帧 PD，等待 APP 关机；网页断联或急停仍会立即切阻尼。
    任意状态 BACK：急停切阻尼（DAMPED，之后可 L2+A 再起身）；WALK 中倾倒（骨盆 z 轴与竖直夹角 > 60°）自动切阻尼。
    手柄断开：趴着 / 阻尼时只提示，等重连；动作中按 pad_lost 处理——damp 立即切阻尼，
    liedown 摇杆当回中，起身 / 趴下照常放完，行走中转 STRAIGHTEN 自动趴下（重连后 BACK 仍可急停）。
    没有技能包（仅仿真）：从站姿开始直接 WALK，L2+A / START 回到站姿，L2+B 不可用。

技能策略与 E_tele 是两个网络，不能做 z 插值：切换时两边各算一次，关节目标在 blend_s 内线性过渡，
上一步 action 记过渡后的值（两个包的 default_joint_pos / action_scale 必须相同）。
PD 增益跟着策略走（各自包里 deploy.yaml 的 pd），过渡期间同样线性插值。
每次切状态（及拒绝起身）播一句语音提示，见 voice.py。
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from apps.teleop.command import BACK, DH_RANGE, DH_RATE, FWD, SPEED_RANGE, SPEED_STEP, TURN, fill_goal, ramp, side_speed
from apps.teleop.gamepad import Pad
from apps.teleop.lying_pose import is_folded_prone
from apps.teleop.voice import Silent
from runtime import Backend, MuJoCoBackend, Package, Session, SkillModel, TeleGoal
from runtime.body_ref import BodyRef
from runtime.primebot_io import vendor_mc_running

LYING, FOLDED, UNFOLD, GETUP, WALK, STRAIGHTEN, LIEDOWN, FOLD, DANCE, DAMPED = (
    "LYING", "FOLDED", "UNFOLD", "GETUP", "WALK", "STRAIGHTEN", "LIEDOWN", "FOLD", "DANCE", "DAMPED"
)
FALL_UPRIGHT = 0.5
LYING_UPRIGHT = 0.5  # 真机起身前骨盆 z 轴与竖直的余弦须低于它：起身参考从趴姿开始
SETTLE_GYRO = 0.3  # rad/s：STRAIGHTEN 里骨盆角速度低于它才算站稳
HANDOFF_JOINT_MAX = 0.5  # rad：技能交接点单关节最大允许差异
START_POSE_JOINT_MAX = 0.7  # rad：阻尼后只允许从接近技能首帧的姿态重启
READY_SAY = "准备就绪，按 L2 加 A 起身"
WALK_MSG = "左摇杆 vx / vy，右摇杆 yaw，十字键 ↑↓ 高度、←→ 速度，Y 高度回正；L2+X 跳舞，L2+B 趴下"
PAD_REMIND_S = 30.0  # 趴着 / 阻尼且没手柄时，每隔这么久提醒一次连接手柄


class Teleop:
    def __init__(self, base: Package, skill: Package | None, robot: Backend, getup_ref: Path, liedown_ref: Path,
                 speed_scale: float = 1.0, blend_s: float = 0.4, settle_s: float = 1.5, voice: Silent | None = None,
                 pad_lost: str = "damp", dance: Package | None = None, dance_ref: Path | None = None,
                 unfold_skill: Package | None = None, unfold_ref: Path | None = None,
                 fold_skill: Package | None = None, fold_ref: Path | None = None):
        """skill：起身 / 趴下技能包（export_package.py 导出的单段动作包）；None 时只能在仿真里从站姿行走。"""
        if pad_lost not in ("damp", "liedown"):
            raise ValueError(f"pad_lost 只能是 damp / liedown：{pad_lost}")
        self.sim = isinstance(robot, MuJoCoBackend)
        if skill is None and not self.sim:
            raise ValueError("真机需要起身 / 趴下技能包")
        if skill is not None and not (np.allclose(skill.default_joint_pos, base.default_joint_pos)
                                      and np.allclose(skill.action_scale, base.action_scale)):
            raise ValueError("技能包与基础模型包的 default_joint_pos / action_scale 不同，不能共用观测历史")
        self.base, self.robot = base, robot
        self.s = Session(base, robot)
        self.goal = TeleGoal(base)
        self.pd_base = (base.kp, base.kd)
        self.has_skill = skill is not None
        if self.has_skill:
            self.skill = SkillModel(skill)
            self.getup = BodyRef(skill, getup_ref)
            self.liedown = BodyRef(skill, liedown_ref)
            self.pd_skill = (skill.kp, skill.kd)
            if not (np.allclose(base.kp, skill.kp) and np.allclose(base.kd, skill.kd)):
                print(f"[teleop] 技能包 PD 与基础模型不同（{skill.spec['pd'].get('preset', '?')}），起身 / 趴下用技能包的增益，切换时插值", flush=True)
        self.has_unfold = unfold_skill is not None
        self.has_fold = fold_skill is not None
        if (self.has_unfold or self.has_fold) and not self.has_skill:
            raise ValueError("展开 / 折叠技能必须与现有起身 / 趴下技能一起使用")
        if self.has_unfold != (unfold_ref is not None):
            raise ValueError("unfold_skill 和 unfold_ref 必须同时提供")
        if self.has_fold != (fold_ref is not None):
            raise ValueError("fold_skill 和 fold_ref 必须同时提供")
        if self.has_unfold:
            self._check_skill_compat(unfold_skill, "展开")
            self.unfold_skill = SkillModel(unfold_skill)
            self.unfold_ref = BodyRef(unfold_skill, unfold_ref)
            self.pd_unfold = (unfold_skill.kp, unfold_skill.kd)
            if not is_folded_prone(base.joint_names, self.unfold_ref.joint_pos[0]):
                raise ValueError("展开参考首帧不是深度折叠姿态")
            if is_folded_prone(base.joint_names, self.unfold_ref.joint_pos[-1]):
                raise ValueError("展开参考末帧仍是深度折叠姿态")
            err = float(np.abs(self.unfold_ref.joint_pos[-1] - self.getup.joint_pos[0]).max())
            if err > HANDOFF_JOINT_MAX:
                raise ValueError(f"UNFOLD→GETUP 交接关节差 {err:.3f} rad，超过 {HANDOFF_JOINT_MAX:.3f}")
        if self.has_fold:
            self._check_skill_compat(fold_skill, "折叠")
            self.fold_skill = SkillModel(fold_skill)
            self.fold_ref = BodyRef(fold_skill, fold_ref)
            self.pd_fold = (fold_skill.kp, fold_skill.kd)
            if is_folded_prone(base.joint_names, self.fold_ref.joint_pos[0]):
                raise ValueError("折叠参考首帧已是深度折叠姿态")
            if not is_folded_prone(base.joint_names, self.fold_ref.joint_pos[-1]):
                raise ValueError("折叠参考末帧不是深度折叠姿态")
            err = float(np.abs(self.fold_ref.joint_pos[0] - self.liedown.joint_pos[-1]).max())
            if err > HANDOFF_JOINT_MAX:
                raise ValueError(f"LIEDOWN→FOLD 交接关节差 {err:.3f} rad，超过 {HANDOFF_JOINT_MAX:.3f}")
        self.has_dance = dance is not None
        if self.has_dance:
            if dance_ref is None:
                raise ValueError("给了舞蹈技能包但没有 dance_ref")
            if not (np.allclose(dance.default_joint_pos, base.default_joint_pos)
                    and np.allclose(dance.action_scale, base.action_scale)):
                raise ValueError("舞蹈包与基础模型包的 default_joint_pos / action_scale 不同，不能共用观测历史")
            self.dance_skill = SkillModel(dance)
            self.dance_ref = BodyRef(dance, dance_ref)
            self.pd_dance = (dance.kp, dance.kd)
        self.dt = base.control_dt
        self.speed = speed_scale
        self.blend_n = max(1, int(round(blend_s / self.dt)))
        self.settle_s = settle_s
        self.voice = voice or Silent()
        self.mode, self.msg = LYING, "趴在地上：L2+A 起身"
        self.cmd = np.zeros(6)
        self.dh = 0.0
        self.i = 0
        self.t_mode = 0.0
        self.fade = None  # (旧策略, 旧 PD, 剩余步数)
        self.after_straighten: str | None = None
        self.fold_after_liedown = False
        self.pad_lost = pad_lost
        self.pad_ok: bool | None = None  # 上一拍手柄是否连着，None = 还没 tick 过
        self.t_remind = 0.0
        self.target = self.kp = None  # 本周期下发的 PD 目标与 kp（不在跟踪为 None），给记录用
        self.recorder = None  # recorder.Recorder：每周期 tick 之后记一行
        if self.sim:
            self.place_lying() if self.has_skill else self.place_standing()

    def _check_skill_compat(self, pkg: Package, label: str) -> None:
        if not (pkg.joint_names == self.base.joint_names
                and np.allclose(pkg.default_joint_pos, self.base.default_joint_pos)
                and np.allclose(pkg.action_scale, self.base.action_scale)
                and np.isclose(pkg.control_dt, self.base.control_dt)):
            raise ValueError(f"{label}技能包与基础模型包的关节/默认姿态/动作缩放/控制周期不兼容")

    # ---- 状态切换
    def _enter(self, mode: str, msg: str, say: str | None = None) -> None:
        self.mode, self.msg, self.t_mode = mode, msg, 0.0
        print(f"[teleop] {mode:10s} {msg}", flush=True)
        if say:
            self.voice.say(say)

    def place_lying(self, announce: bool = True) -> None:
        """仿真：把机器人摆成起身参考首帧（趴姿）。"""
        r = self.getup
        self.robot.set_state(r.root_pos[0], r.root_quat[0], r.joint_pos[0])
        if announce:
            self._enter(LYING, "趴在地上：L2+A 起身", READY_SAY)

    def place_folded(self, announce: bool = True) -> None:
        """仿真：把机器人摆成展开参考首帧（深度折叠）。"""
        if not self.has_unfold:
            raise RuntimeError("未安装展开技能")
        r = self.unfold_ref
        self.robot.set_state(r.root_pos[0], r.root_quat[0], r.joint_pos[0])
        if announce:
            self._enter(FOLDED, "已折叠：按起身将自动展开并起身", "已折叠")

    def place_standing(self) -> None:
        """仿真摆成默认站姿，直接行走。"""
        self.robot.reset()
        self.s.begin()
        self.cmd[:], self.dh, self.fade = 0.0, 0.0, None
        msg = WALK_MSG if self.has_skill else WALK_MSG.replace("，L2+B 趴下", "（没有技能包，不能趴下）")
        self._enter(WALK, msg, "可以行走")

    def damp(self, why: str, say: str | None = None) -> None:
        self.robot.damp()
        self.fade = None
        self._enter(DAMPED, f"{why}，已切阻尼：L2+A 起身" + ("，START 重置" if self.sim else ""), f"{say or why}，已切阻尼")

    def _low_level_ready(self, action: str) -> bool:
        if not self.sim and vendor_mc_running():
            self.msg = f"厂商运控 mc 在运行（APP 没选「低层运控开发」），不{action}"
            print(f"[teleop] {self.msg}", flush=True)
            self.voice.say(f"高层运控正在运行，不能{action}")
            return False
        return True

    def _start_flat_getup(self, previous=None) -> None:
        """从平趴开始现有 GETUP；previous 非 None 表示刚放完 UNFOLD，无缝过渡。"""
        r = self.getup
        if previous is None:
            if self.sim:
                self.place_lying(announce=False)
                self.robot.set_pd(*self.pd_skill)
            else:
                self.robot.set_pd(*self.pd_skill)
                self.robot.reset(r.joint_pos[0])
            self.s.begin()
        else:
            model, old_ref, pd = previous
            self.fade = (self._ref_policy(model, old_ref, old_ref.T - 1), pd, self.blend_n)
        r.start(self.robot.torso_quat())
        self.i = 0
        self._enter(GETUP, f"起身 {(r.T - 1) * self.dt:.1f} s", "展开完成，开始起身" if previous else "开始起身")

    def _start_getup(self) -> None:
        if not self.has_skill:
            return self.place_standing()
        if not self._low_level_ready("起身"):
            return
        sensed = self.robot.sense()
        if not self.sim and self.robot.upright() > LYING_UPRIGHT:
            self.msg = f"upright {self.robot.upright():+.2f}：不是趴着，不起身"
            print(f"[teleop] {self.msg}", flush=True)
            self.voice.say("没有趴着，不能起身")
            return
        folded = self.mode == FOLDED or is_folded_prone(self.base.joint_names, sensed.joint_pos)
        if not folded:
            err = float(np.abs(sensed.joint_pos - self.getup.joint_pos[0]).max())
            if not self.sim and err > START_POSE_JOINT_MAX:
                self.msg = f"当前既不是深折叠也不接近平趴（最大关节差 {err:.2f} rad），拒绝起身"
                print(f"[teleop] {self.msg}", flush=True)
                self.voice.say("当前姿态不明，不能起身")
                return
            return self._start_flat_getup()
        if not self.has_unfold:
            self.msg = "检测到深度折叠：UNFOLD 展开技能尚未安装，拒绝进入平趴起身"
            print(f"[teleop] {self.msg}", flush=True)
            self.voice.say("展开技能尚未安装，不能起身")
            return
        r = self.unfold_ref
        self.voice.say("准备展开")
        self.robot.set_pd(*self.pd_unfold)
        if not self.sim:
            self.robot.reset(r.joint_pos[0])
        self.s.begin()
        r.start(self.robot.torso_quat())
        self.i = 0
        self._enter(UNFOLD, f"展开 {(r.T - 1) * self.dt:.1f} s；完成后自动起身", "开始展开")

    def _start_liedown(self, fold_after: bool = False) -> None:
        self.liedown.start(self.robot.torso_quat())
        self.i = 0
        self.fold_after_liedown = fold_after
        self.fade = (self._tele_policy, self.pd_base, self.blend_n)
        suffix = "；完成后自动折叠" if fold_after else ""
        self._enter(LIEDOWN, f"趴下 {(self.liedown.T - 1) * self.dt:.1f} s{suffix}", "开始趴下")

    def _start_fold(self, previous=None) -> None:
        """从平趴开始 FOLD；previous 非 None 表示刚放完 LIEDOWN。"""
        if not self.has_fold:
            self.msg = "FOLD 折叠技能尚未安装"
            print(f"[teleop] {self.msg}", flush=True)
            self.voice.say("折叠技能尚未安装")
            return
        if not self._low_level_ready("折叠"):
            return
        r = self.fold_ref
        if previous is None:
            sensed = self.robot.sense()
            if is_folded_prone(self.base.joint_names, sensed.joint_pos):
                if self.sim:
                    return self.place_folded()
                self.robot.set_pd(*self.pd_fold)
                self.robot.reset(r.joint_pos[-1])
                self.s.begin()
                r.start(self.robot.torso_quat(), r.T - 1)
                self.i = r.T - 1
                return self._enter(FOLDED, "已折叠，保持姿态，可在 APP 关机", "已折叠，可以关机")
            if not self.sim and self.robot.upright() > LYING_UPRIGHT:
                self.msg = "机器人尚未趴平，拒绝直接执行折叠"
                print(f"[teleop] {self.msg}", flush=True)
                self.voice.say("机器人尚未趴平，不能折叠")
                return
            err = float(np.abs(sensed.joint_pos - r.joint_pos[0]).max())
            if not self.sim and err > START_POSE_JOINT_MAX:
                self.msg = f"当前不接近 FOLD 平趴首帧（最大关节差 {err:.2f} rad），拒绝折叠"
                print(f"[teleop] {self.msg}", flush=True)
                self.voice.say("当前姿态不明，不能折叠")
                return
            self.robot.set_pd(*self.pd_fold)
            if self.sim:
                self.robot.set_state(r.root_pos[0], r.root_quat[0], r.joint_pos[0])
            else:
                self.robot.reset(r.joint_pos[0])
            self.s.begin()
        else:
            model, old_ref, pd = previous
            self.fade = (self._ref_policy(model, old_ref, old_ref.T - 1), pd, self.blend_n)
        r.start(self.robot.torso_quat())
        self.i = 0
        self._enter(FOLD, f"折叠 {(r.T - 1) * self.dt:.1f} s；完成后保持等待关机", "开始折叠")

    def _start_dance(self) -> None:
        """只从已经停稳的 STRAIGHTEN 进入；技能与基础模型之间插值关节目标与 PD。"""
        self.dance_ref.start(self.robot.torso_quat())
        self.i = 0
        self.fade = (self._tele_policy, self.pd_base, self.blend_n)
        self._enter(DANCE, f"舞蹈 {(self.dance_ref.T - 1) * self.dt:.1f} s", "开始跳舞")

    def adjust_speed(self, steps: int) -> None:
        s = float(np.clip(round(self.speed + steps * SPEED_STEP, 2), *SPEED_RANGE))
        if steps == 0:
            return
        if s == self.speed:
            self.voice.say("已是最高速度" if steps > 0 else "已是最低速度")
            return
        self.speed = s
        print(f"[teleop] 速度系数 {s:.2f}", flush=True)
        self.voice.say(f"速度 {s:g}")

    # ---- 策略
    def _ref_policy(self, model: SkillModel, ref: BodyRef, i: int):
        return lambda obs: model.act(obs, ref.goal(i, self.robot.torso_quat()))

    def _tele_policy(self, obs: np.ndarray):
        fill_goal(self.goal, self.cmd)
        return self.s.model.act_tele(obs, self.goal)

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
        self.target, self.kp = target, kp

    def _idle(self) -> None:
        """不在跟踪：真机不发 PD（阻尼或未启动），只按控制周期等待；仿真里阻尼状态下推进物理。"""
        if self.sim:
            self.robot.damp()
            self.robot.step(self.robot.sense().joint_pos)
        else:
            time.sleep(self.dt)

    # ---- 每个控制周期
    def tick(self, pad: Pad, edges: set[str]) -> None:
        self.target = self.kp = None
        try:
            self._tick(pad, edges)
        except RuntimeError as e:  # 真机后端断流 / 超时已自行切阻尼
            self._enter(DAMPED, f"后端故障：{e}", "后端故障，已切阻尼")
        self.t_mode += self.dt
        if self.recorder is not None:
            self.recorder.step(self, pad, edges)

    def _pad_changed(self, pad: Pad) -> None:
        idle = self.mode in (LYING, FOLDED, DAMPED)
        first, self.pad_ok = self.pad_ok is None, pad.connected
        if pad.connected:
            if first:
                if not self.sim:
                    self.voice.say(READY_SAY)
            else:
                self.voice.say("手柄已连接" + ("，按 L2 加 A 起身" if idle else ""))
        elif idle:
            self.t_remind = PAD_REMIND_S
            self.voice.say("请连接手柄" if first else "手柄断开，请重新连接")
        elif self.pad_lost == "liedown" and self.mode != WALK:  # 行走在 _tick 里转 STRAIGHTEN 时播
            self.voice.say("手柄断开，起身后自动趴下" if self.mode in (UNFOLD, GETUP) else "手柄断开")

    def _tick(self, pad: Pad, edges: set[str]) -> None:
        if pad.connected != self.pad_ok:
            self._pad_changed(pad)
        if not pad.connected and self.mode in (LYING, FOLDED, DAMPED):
            self.t_remind -= self.dt
            if self.t_remind <= 0:
                self.t_remind = PAD_REMIND_S
                self.voice.say("请连接手柄")
        if not pad.connected and self.mode not in (LYING, FOLDED, DAMPED):
            if self.pad_lost == "damp" or not self.has_skill:
                return self.damp("手柄断开")
            if self.mode == WALK:
                self._enter(STRAIGHTEN, "手柄断开：站直后自动趴下", "手柄断开，自动趴下")
        if ("BACK" in edges or "DAMP" in edges) and self.mode not in (LYING, DAMPED):
            return self.damp("急停（BACK）", "急停")
        if self.sim and "START" in edges:
            return self.place_lying() if self.has_skill else self.place_standing()
        if "LEFT" in edges or "RIGHT" in edges:
            self.adjust_speed(("RIGHT" in edges) - ("LEFT" in edges))

        if self.mode in (LYING, DAMPED):
            if (pad.l2 and "A" in edges) or "GETUP" in edges:
                return self._start_getup()
            if "FOLD" in edges:
                return self._start_fold()
            return self._idle()

        if self.mode == FOLDED:
            if (pad.l2 and "A" in edges) or "GETUP" in edges:
                return self._start_getup()
            if self.has_fold:
                model, ref, pd, frame = self.fold_skill, self.fold_ref, self.pd_fold, self.fold_ref.T - 1
            else:
                model, ref, pd, frame = self.unfold_skill, self.unfold_ref, self.pd_unfold, 0
            self._act(self._ref_policy(model, ref, frame), pd)
            return

        if self.mode in (UNFOLD, GETUP, LIEDOWN, FOLD):
            if self.mode == UNFOLD:
                ref, model, pd = self.unfold_ref, self.unfold_skill, self.pd_unfold
            elif self.mode == FOLD:
                ref, model, pd = self.fold_ref, self.fold_skill, self.pd_fold
            else:
                ref, model, pd = (self.getup, self.skill, self.pd_skill) if self.mode == GETUP \
                    else (self.liedown, self.skill, self.pd_skill)
            self._act(self._ref_policy(model, ref, self.i), pd)
            self.i += 1
            if self.i < ref.T - 1:
                return
            if self.mode == UNFOLD:
                return self._start_flat_getup((model, ref, pd))
            if self.mode == LIEDOWN:
                if self.fold_after_liedown:
                    self.fold_after_liedown = False
                    return self._start_fold((model, ref, pd))
                self.robot.damp()
                return self._enter(LYING, "趴下完成，已切阻尼：L2+A 起身", "趴下完成，已切阻尼")
            if self.mode == FOLD:
                self.i = ref.T - 1
                self.fade = None
                return self._enter(FOLDED, "折叠完成，正在保持姿态；现在可在 APP 关机", "折叠完成，可以关机")
            self.cmd[:], self.dh = 0.0, 0.0
            self.fade = (self._ref_policy(model, ref, ref.T - 1), pd, self.blend_n)
            return self._enter(WALK, WALK_MSG, "起身完成，可以行走")

        if self.mode == DANCE:
            self._act(self._ref_policy(self.dance_skill, self.dance_ref, self.i), self.pd_dance)
            self.i += 1
            if self.robot.upright() < FALL_UPRIGHT:
                return self.damp("舞蹈中倾倒")
            if self.i < self.dance_ref.T - 1:
                return
            self.cmd[:], self.dh = 0.0, 0.0
            self.fade = (self._ref_policy(self.dance_skill, self.dance_ref, self.dance_ref.T - 1),
                         self.pd_dance, self.blend_n)
            return self._enter(WALK, WALK_MSG, "舞蹈完成，已回到站立")

        if self.mode == WALK:
            if "FOLD" in edges:
                if not (self.has_skill and self.has_fold):
                    self.msg = "需要 LIEDOWN 和 FOLD 两个技能才能折叠"
                    self.voice.say("折叠技能尚未安装")
                else:
                    self.after_straighten = "fold"
                    return self._enter(STRAIGHTEN, "站直，准备趴下并折叠", "站直，准备折叠")
            if (pad.l2 and "B" in edges) or "LIEDOWN" in edges:
                if not self.has_skill:
                    self.msg = "没有技能包，不能趴下"
                    self.voice.say("没有技能包，不能趴下")
                else:
                    self.after_straighten = "liedown"
                    return self._enter(STRAIGHTEN, "站直", "站直，准备趴下")
            if (pad.l2 and "X" in edges) or "DANCE" in edges:
                if not self.has_dance:
                    self.msg = "没有舞蹈技能包"
                    self.voice.say("没有舞蹈技能包")
                else:
                    self.after_straighten = "dance"
                    return self._enter(STRAIGHTEN, "站直", "站直，准备跳舞")
            if "Y" in edges:
                self.dh = 0.0
            self.dh = float(np.clip(self.dh + DH_RATE * self.dt * (pad.buttons["UP"] - pad.buttons["DOWN"]), *DH_RANGE))
            target = np.array([self.speed * (FWD if pad.ly > 0 else BACK) * pad.ly, side_speed(pad.lx, self.speed),
                               self.speed * TURN * pad.rx, self.dh, 0.0, 0.0])
        else:  # STRAIGHTEN
            target = np.zeros(6)
            self.dh = 0.0
        self.cmd = ramp(self.cmd, target, self.dt)
        self._act(self._tele_policy, self.pd_base)
        if self.robot.upright() < FALL_UPRIGHT:
            return self.damp("倾倒")
        if self.mode == STRAIGHTEN and self.t_mode >= self.settle_s and np.abs(self.cmd).max() < 1e-3 \
                and np.linalg.norm(self.robot.sense().base_ang_vel) < SETTLE_GYRO:
            action, self.after_straighten = self.after_straighten, None
            self._start_dance() if action == "dance" else self._start_liedown(fold_after=action == "fold")

    def status(self) -> list[tuple[str, str]]:
        c = self.cmd
        rows = [("mode", self.mode), ("", self.msg)]
        if self.mode in (WALK, STRAIGHTEN):
            rows += [("vx vy wz", f"{c[0]:+.2f} {c[1]:+.2f} {c[2]:+.2f}"), ("dh m", f"{c[3]:+.3f}"), ("speed", f"{self.speed:.1f}")]
        elif self.mode in (FOLDED, UNFOLD, GETUP, LIEDOWN, FOLD, DANCE):
            ref = self.unfold_ref if self.mode == UNFOLD or (self.mode == FOLDED and not self.has_fold) \
                else self.fold_ref if self.mode in (FOLD, FOLDED) else \
                self.dance_ref if self.mode == DANCE else self.getup if self.mode == GETUP else self.liedown
            rows.append(("t", f"{self.i * self.dt:.1f} / {(ref.T - 1) * self.dt:.1f} s"))
        return rows
