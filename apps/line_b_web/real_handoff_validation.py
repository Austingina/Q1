"""线路 B 起身→基础策略的一次性零速交接验收。

这不是网页控制器，也没有行走或舞蹈输入。固定流程是：

    标准平贴俯卧 -> 起身 -> 10 s 零速策略交接 -> 3 s 零速观察 -> 贴地 -> 阻尼

缺省仅运行 MuJoCo。真机只读预检使用 ``--backend real --check-only``。
真机下发必须再加 ``--execute-real-handoff``，并在终端输入固定确认短语。
每一个 50 Hz 策略控制帧都写入 npz；任何门禁失败、后端故障或 Ctrl-C 都切阻尼。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from apps import ref  # noqa: E402
from apps.line_b_web.machine import (  # noqa: E402
    DAMPED,
    GETUP,
    LIEDOWN,
    LYING,
    STAND_HOLD,
    STRAIGHTEN,
    WALK,
    WALK_HANDOFF,
    WALK_SETTLE,
    HandoffConfig,
    LineBMachine,
)
from apps.teleop.gamepad import Pad  # noqa: E402
from apps.teleop.lying_pose import is_folded_prone  # noqa: E402
from runtime import MuJoCoBackend, Package, PrimeBotBackend  # noqa: E402
from runtime.body_ref import BodyRef  # noqa: E402
from runtime.primebot_io import realtime, vendor_mc_running  # noqa: E402

DEFAULT_BASE = PROJECT / "weights" / "dual_v3b"
DEFAULT_SKILL = PROJECT / "weights" / "getup_vpd_ft"
DEFAULT_DANCE = PROJECT / "weights" / "q1_smooth_dance"
DRY_TOPIC = "/kabutack/dry_run/joint/command"
CONFIRM_TEXT = "线路B零速交接"
LYING_UPRIGHT = 0.5
DEFAULT_START_ERROR = 0.7
INFERENCE_MEAN_LIMIT_MS = 15.0
INFERENCE_P99_LIMIT_MS = 18.0
INFERENCE_BENCHMARK_WARMUP = 20
INFERENCE_BENCHMARK_SAMPLES = 200
POLICY_CORE = 6  # Q1 A72 大核；0–3 是 A53，4/5 专供 ROS 收发进程
POLICY_PRIO = 60


class ValidationFailure(RuntimeError):
    pass


@dataclass(frozen=True)
class PoseCheck:
    passed: bool
    upright: float
    max_start_error: float
    worst_joint: str
    folded: bool
    reason: str = ""


@dataclass(frozen=True)
class InferenceTiming:
    passed: bool
    skill_mean_ms: float
    skill_p99_ms: float
    skill_max_ms: float
    base_mean_ms: float
    base_p99_ms: float
    base_max_ms: float
    mean_limit_ms: float
    p99_limit_ms: float
    samples: int
    reason: str = ""


def check_prone_pose(skill: Package, getup: BodyRef, sensed, upright: float,
                     max_start_error: float = DEFAULT_START_ERROR) -> PoseCheck:
    dq = np.asarray(sensed.joint_pos) - getup.joint_pos[0]
    worst = int(np.argmax(np.abs(dq)))
    error = float(abs(dq[worst]))
    folded = bool(is_folded_prone(skill.joint_names, sensed.joint_pos))
    reason = ""
    if upright > LYING_UPRIGHT:
        reason = f"upright {upright:+.3f} > {LYING_UPRIGHT:.3f}：不是俯卧"
    elif folded:
        reason = "检测到深度折叠：该姿态属于线路 A"
    elif error > max_start_error:
        reason = (f"{skill.joint_names[worst]} 与起身首帧差 {error:.3f} rad"
                  f" > {max_start_error:.3f} rad")
    return PoseCheck(not reason, float(upright), error, skill.joint_names[worst], folded, reason)


def evaluate_inference_timing(skill_ms: np.ndarray, base_ms: np.ndarray) -> InferenceTiming:
    skill_ms, base_ms = np.asarray(skill_ms, dtype=float), np.asarray(base_ms, dtype=float)
    skill_mean, base_mean = float(skill_ms.mean()), float(base_ms.mean())
    skill_p99, base_p99 = float(np.percentile(skill_ms, 99)), float(np.percentile(base_ms, 99))
    reason = ""
    if max(skill_mean, base_mean) > INFERENCE_MEAN_LIMIT_MS:
        reason = (f"平均推理 {max(skill_mean, base_mean):.1f} ms"
                  f" > {INFERENCE_MEAN_LIMIT_MS:.1f} ms")
    elif max(skill_p99, base_p99) > INFERENCE_P99_LIMIT_MS:
        reason = (f"p99 推理 {max(skill_p99, base_p99):.1f} ms"
                  f" > {INFERENCE_P99_LIMIT_MS:.1f} ms")
    return InferenceTiming(
        passed=not reason,
        skill_mean_ms=skill_mean,
        skill_p99_ms=skill_p99,
        skill_max_ms=float(skill_ms.max()),
        base_mean_ms=base_mean,
        base_p99_ms=base_p99,
        base_max_ms=float(base_ms.max()),
        mean_limit_ms=INFERENCE_MEAN_LIMIT_MS,
        p99_limit_ms=INFERENCE_P99_LIMIT_MS,
        samples=len(skill_ms),
        reason=reason,
    )


def benchmark_inference(machine: LineBMachine, samples: int = INFERENCE_BENCHMARK_SAMPLES) -> InferenceTiming:
    """按真实 50 Hz 节拍测量单次策略推理；不调 robot.step/reset。

    FIFO 进程若背靠背占满 CPU，会触发 Linux ``sched_rt_runtime_us`` 的周期性
    50 ms 节流；真实控制循环每 20 ms 会休眠剩余时间，因此基准也必须按该节拍运行。
    """
    machine.s.begin()
    machine.getup.start(machine.robot.torso_quat())
    obs = machine.s.obs()
    skill_goal = machine.getup.goal(machine.getup.T - 1, machine.robot.torso_quat())

    def measure(call) -> np.ndarray:
        # ORT 在模型刚加载后的少数调用里仍可能做惰性初始化。所有预热都发生在
        # 电机 reset/step 和终端动作确认之前；实际动作使用已预热的同一会话。
        values = []
        next_tick = time.monotonic()
        total = INFERENCE_BENCHMARK_WARMUP + samples
        for i in range(total):
            t0 = time.perf_counter()
            call()
            elapsed_ms = (time.perf_counter() - t0) * 1e3
            if i >= INFERENCE_BENCHMARK_WARMUP:
                values.append(elapsed_ms)
            next_tick += machine.dt
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
        return np.asarray(values)

    skill_ms = measure(lambda: machine.skill.act(obs, skill_goal))
    base_ms = measure(lambda: machine._zero_speed_base_policy(obs))
    # 基础模型的 z_prev 被基准调用更新；真正回合开始前重置。
    machine.s.begin()
    return evaluate_inference_timing(skill_ms, base_ms)


class FrameLog:
    """记录每次 machine.tick 后的传感器、指令和交接门禁值。"""

    _scalar_metrics = (
        "initial_policy_gap",
        "max_policy_gap",
        "max_raw_target_step",
        "max_applied_target_step",
        "max_tracking_error",
        "min_upright",
        "max_gyro",
        "final_base_residual",
    )

    def __init__(self, joint_count: int):
        self.nj = int(joint_count)
        keys = (
            "t", "mode", "frame", "handoff_frame", "settle_frame", "q", "qd", "gyro", "grav",
            "target", "action", "kp", "kd", "cmd", "upright", "tracking", "tick_ms",
        ) + self._scalar_metrics
        self.rows: dict[str, list] = {key: [] for key in keys}

    def add(self, machine: LineBMachine, tick_ms: float) -> None:
        sensed = machine.robot.sense()
        nan_joints = np.full(self.nj, np.nan, dtype=np.float64)
        target = nan_joints if machine.target is None else np.asarray(machine.target, dtype=np.float64)
        action = np.asarray(getattr(machine.s, "last_action", nan_joints), dtype=np.float64)
        kp = nan_joints if machine.kp is None else np.asarray(machine.kp, dtype=np.float64)
        kd = nan_joints if machine.kd is None else np.asarray(machine.kd, dtype=np.float64)
        tracking = float("nan") if machine.target is None else float(np.abs(target - sensed.joint_pos).max())
        values = {
            "t": time.monotonic(),
            "mode": machine.mode,
            "frame": int(machine.i),
            "handoff_frame": int(machine.handoff_i),
            "settle_frame": int(machine.handoff_settle_i),
            "q": sensed.joint_pos,
            "qd": sensed.joint_vel,
            "gyro": sensed.base_ang_vel,
            "grav": sensed.projected_gravity,
            "target": target,
            "action": action,
            "kp": kp,
            "kd": kd,
            "cmd": machine.cmd,
            "upright": machine.robot.upright(),
            "tracking": tracking,
            "tick_ms": tick_ms,
        }
        metrics = asdict(machine.handoff_metrics)
        for key in self._scalar_metrics:
            values[key] = metrics[key]
        for key, value in values.items():
            self.rows[key].append(np.asarray(value).copy() if isinstance(value, np.ndarray) else value)

    def save(self, path: Path, **meta) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        arrays = {key: np.asarray(values) for key, values in self.rows.items()}
        np.savez(path, **arrays, **{key: np.asarray(value) for key, value in meta.items()})
        print(f"[handoff-validation] 记录 {len(self.rows['t'])} 帧 → {path}", flush=True)


class OneShotRunner:
    def __init__(self, machine: LineBMachine, log: FrameLog):
        self.machine, self.log = machine, log
        self.dt = machine.dt
        self._tick_count = 0

    def tick(self, edges: set[str] | None = None) -> None:
        t0 = time.perf_counter()
        self.machine.tick(Pad(), edges or set())
        tick_ms = (time.perf_counter() - t0) * 1e3
        self.log.add(self.machine, tick_ms)
        self._tick_count += 1
        if float(np.abs(self.machine.cmd).max()) > 1e-12:
            self.machine.damp("一次性验收出现非零速指令")
            raise ValidationFailure("出现非零速指令")
        if self.machine.mode == DAMPED:
            raise ValidationFailure(self.machine.msg)

    def until(self, target: str, allowed: set[str], timeout_s: float) -> None:
        steps = max(1, int(round(timeout_s / self.dt)))
        report_every = max(1, int(round(1.0 / self.dt)))
        for _ in range(steps):
            self.tick()
            if self.machine.mode == target:
                return
            if self.machine.mode not in allowed:
                raise ValidationFailure(f"期望 {target}，却进入 {self.machine.mode}：{self.machine.msg}")
            if self._tick_count % report_every == 0:
                sensed = self.machine.robot.sense()
                print(f"[handoff-validation] {self.machine.mode:12s}  upright {self.machine.robot.upright():+.3f}"
                      f"  gyro {np.linalg.norm(sensed.base_ang_vel):.3f} rad/s", flush=True)
        raise ValidationFailure(f"等待 {target} 超时（{timeout_s:.1f} s）")

    def stable_gate(self, label: str) -> None:
        sensed = self.machine.robot.sense()
        upright = self.machine.robot.upright()
        gyro = float(np.linalg.norm(sensed.base_ang_vel))
        target = self.machine.target
        tracking = float("inf") if target is None else float(np.abs(target - sensed.joint_pos).max())
        cfg = self.machine.handoff
        if upright < cfg.upright_min:
            raise ValidationFailure(f"{label}: upright {upright:.3f} < {cfg.upright_min:.3f}")
        if gyro > cfg.gyro_max:
            raise ValidationFailure(f"{label}: gyro {gyro:.3f} > {cfg.gyro_max:.3f} rad/s")
        if tracking > cfg.tracking_max:
            raise ValidationFailure(f"{label}: tracking {tracking:.3f} > {cfg.tracking_max:.3f} rad")

    def run(self) -> dict:
        print("[handoff-validation] 固定流程：起身 → 10 s 零速交接 → 3 s 零速观察 → 贴地", flush=True)
        self.tick({"GETUP"})
        if self.machine.mode != GETUP:
            raise ValidationFailure(f"起身未启动：{self.machine.msg}")
        self.until(STAND_HOLD, {GETUP}, 15.0)
        self.stable_gate("起身末帧")

        self.tick({"WALK"})
        if self.machine.mode != WALK_HANDOFF:
            raise ValidationFailure(f"零速交接未启动：{self.machine.msg}")
        timeout = self.machine.handoff.duration_s + self.machine.handoff.settle_s + 5.0
        self.until(WALK, {WALK_HANDOFF, WALK_SETTLE}, timeout)
        self.stable_gate("零速交接结束")

        # WALK 只是交接通过的内部标志；Pad 始终为零，下一拍立即请求贴地。
        self.tick({"LIEDOWN"})
        if self.machine.mode != STRAIGHTEN:
            raise ValidationFailure(f"贴地请求未接受：{self.machine.msg}")
        self.until(LYING, {STRAIGHTEN, LIEDOWN}, 15.0)

        metrics = self.machine.handoff_metrics.as_dict()
        if metrics["failure"]:
            raise ValidationFailure(metrics["failure"])
        return {"passed": True, "final_mode": self.machine.mode, "handoff": metrics}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Q1 线路 B 一次性真机零速交接验收（无网页/无行走/无舞蹈）")
    p.add_argument("--backend", choices=("sim", "real"), default="sim")
    p.add_argument("--check-only", action="store_true", help="真机只读摆姿预检；真实命令话题不建立发布者")
    p.add_argument("--timing-check-only", action="store_true",
                   help="真机 CPU 只读推理时延门禁；使用假话题，不调 reset/step")
    p.add_argument("--execute-real-handoff", action="store_true", help="真机专用二次执行开关；仍需终端输入固定短语")
    p.add_argument("--package", type=Path, default=DEFAULT_BASE)
    p.add_argument("--skill", type=Path, default=DEFAULT_SKILL)
    p.add_argument("--dance", type=Path, default=DEFAULT_DANCE,
                   help="只为复用状态机的包兼容性校验；本程序不会进入舞蹈状态")
    p.add_argument("--getup-ref", default="getup_facedown")
    p.add_argument("--liedown-ref", default="liedown_facedown")
    p.add_argument("--sensor-timeout", type=float, default=10.0)
    p.add_argument("--ramp-s", type=float, default=3.0)
    p.add_argument("--max-start-error", type=float, default=DEFAULT_START_ERROR)
    p.add_argument("--core", type=int, default=POLICY_CORE,
                   help="策略进程绑核（默认 6：A72 大核；收发进程使用 4、5）")
    p.add_argument("--prio", type=int, default=POLICY_PRIO)
    p.add_argument("--log", type=Path, default=None,
                   help="默认 q1/getup/data/handoff_validation_<backend>_<时刻>.npz")
    return p


def validate_cli_mode(a: argparse.Namespace) -> None:
    if a.max_start_error <= 0 or a.ramp_s <= 0 or a.sensor_timeout <= 0:
        raise ValueError("时间参数和 --max-start-error 必须大于 0")
    real_modes = (a.check_only, a.timing_check_only, a.execute_real_handoff)
    if a.backend == "sim" and any(real_modes):
        raise ValueError("真机检查/执行开关只能与 --backend real 同用")
    if a.backend == "real" and sum(map(bool, real_modes)) != 1:
        raise ValueError("真机必须三选一：--check-only、--timing-check-only 或 --execute-real-handoff")


def print_pose(check: PoseCheck, sensed) -> None:
    print(f"[handoff-validation] gravity {np.round(sensed.projected_gravity, 3)}  upright {check.upright:+.3f}", flush=True)
    print(f"[handoff-validation] 起身首帧最大关节差 {check.max_start_error:.3f} rad"
          f"（{check.worst_joint}）  folded={check.folded}", flush=True)


def main() -> None:
    parser = build_parser()
    a = parser.parse_args()
    try:
        validate_cli_mode(a)
    except ValueError as exc:
        parser.error(str(exc))

    base, skill, dance = Package.load(a.package), Package.load(a.skill), Package.load(a.dance)
    getup = BodyRef(skill, ref.find(a.getup_ref))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = a.log or PROJECT / "q1" / "getup" / "data" / f"handoff_validation_{a.backend}_{stamp}.npz"

    if a.backend == "real" and a.check_only:
        # 只读预检还把闲置发布进程放到假话题，真实电机话题不建立发布者。
        robot = PrimeBotBackend(base, cmd_topic=DRY_TOPIC, ramp_s=a.ramp_s,
                                ramp_track_lim=float("inf"), track_lim=float("inf"))
        try:
            robot.wait_sensors(a.sensor_timeout)
            sensed = robot.sense()
            check = check_prone_pose(skill, getup, sensed, robot.upright(), a.max_start_error)
            print_pose(check, sensed)
            if not check.passed:
                raise SystemExit(f"[handoff-validation] 只读预检失败：{check.reason}")
            print("[handoff-validation] 只读预检通过；未发布真实电机指令", flush=True)
            return
        finally:
            robot.close()

    if a.backend == "real" and a.timing_check_only:
        # 时延门禁不需要标准贴卧姿态，也不进入 PD 跟踪状态。
        robot = PrimeBotBackend(base, cmd_topic=DRY_TOPIC, ramp_s=a.ramp_s,
                                ramp_track_lim=float("inf"), track_lim=float("inf"))
        try:
            robot.wait_sensors(a.sensor_timeout)
            sensed = robot.sense()
            check = check_prone_pose(skill, getup, sensed, robot.upright(), a.max_start_error)
            print_pose(check, sensed)
            print(f"[handoff-validation] 时延检查不下发；当前摆姿仅记录：{check.reason or '标准贴卧'}", flush=True)
            print(f"[handoff-validation] 策略进程 {realtime(a.core, a.prio)}", flush=True)
            machine = LineBMachine(
                base, skill, robot, ref.find(a.getup_ref), ref.find(a.liedown_ref),
                dance, dance.root / "motion.npz", speed_scale=0.0, settle_s=0.0,
                experimental_handoff=HandoffConfig(), allow_real_handoff_validation=True,
            )
            timing = benchmark_inference(machine)
            print(json.dumps(asdict(timing), ensure_ascii=False, indent=2), flush=True)
            if not timing.passed:
                raise SystemExit(f"[handoff-validation] 推理时延门禁失败：{timing.reason}")
            print("[handoff-validation] 推理时延门禁通过；未发布真实电机指令", flush=True)
            return
        finally:
            robot.close()

    robot = None
    machine = None
    timing = None
    frame_log = FrameLog(len(base.joint_names))
    outcome = {"passed": False, "final_mode": "NOT_STARTED", "failure": ""}
    try:
        if a.backend == "real":
            if vendor_mc_running():
                raise ValidationFailure("厂商运控 mc 正在运行；拒绝并发下发")
            robot = PrimeBotBackend(base, ramp_s=a.ramp_s)
            robot.wait_sensors(a.sensor_timeout)
            sensed = robot.sense()
            check = check_prone_pose(skill, getup, sensed, robot.upright(), a.max_start_error)
            print_pose(check, sensed)
            if not check.passed:
                raise ValidationFailure(f"真机摆姿门禁失败：{check.reason}")
            print(f"[handoff-validation] 策略进程 {realtime(a.core, a.prio)}", flush=True)
        else:
            robot = MuJoCoBackend(base)

        cfg = HandoffConfig(duration_s=10.0, settle_s=3.0)
        machine = LineBMachine(
            base,
            skill,
            robot,
            ref.find(a.getup_ref),
            ref.find(a.liedown_ref),
            dance,
            dance.root / "motion.npz",
            speed_scale=0.0,
            settle_s=0.0,
            experimental_handoff=cfg,
            allow_real_handoff_validation=a.backend == "real",
        )
        if a.backend == "real":
            timing = benchmark_inference(machine)
            print("[handoff-validation] 动作前单策略推理门禁："
                  f"skill mean/p99 {timing.skill_mean_ms:.1f}/{timing.skill_p99_ms:.1f} ms，"
                  f"base {timing.base_mean_ms:.1f}/{timing.base_p99_ms:.1f} ms", flush=True)
            if not timing.passed:
                raise ValidationFailure(f"动作前推理时延门禁失败：{timing.reason}")
            print("[handoff-validation] 确认：无行走、无舞蹈，只执行一次起身→零速交接→贴地。", flush=True)
            typed = input(f"[handoff-validation] 吊带已兜住、独立急停在手边、1 m 内无人后，输入「{CONFIRM_TEXT}」开始：")
            if typed.strip() != CONFIRM_TEXT:
                raise ValidationFailure("确认短语不匹配，未开始")
        outcome = OneShotRunner(machine, frame_log).run()
        print("[handoff-validation] 通过：已贴地并切阻尼", flush=True)
        print(json.dumps(outcome, ensure_ascii=False, indent=2), flush=True)
    except (KeyboardInterrupt, EOFError):
        outcome["failure"] = "用户中止"
        print("[handoff-validation] 用户中止：切阻尼", flush=True)
    except RuntimeError as exc:
        outcome["failure"] = str(exc)
        print(f"[handoff-validation] 失败：{exc}；切阻尼", flush=True)
    finally:
        if machine is not None:
            outcome["final_mode"] = machine.mode
            machine.robot.damp()
        if isinstance(robot, PrimeBotBackend):
            robot.close()
        elif robot is not None:
            robot.damp()
        if frame_log.rows["t"]:
            frame_log.save(
                log_path,
                backend=a.backend,
                success=bool(outcome.get("passed")),
                failure=outcome.get("failure", ""),
                final_mode=outcome.get("final_mode", ""),
                joint_names=np.asarray(base.joint_names),
                handoff_config=json.dumps(asdict(HandoffConfig()), ensure_ascii=False),
                inference_timing=json.dumps(asdict(timing), ensure_ascii=False) if timing is not None else "",
                flow="GETUP->WALK_HANDOFF(10s)->WALK_SETTLE(3s)->LIEDOWN->LYING",
            )

    if not outcome.get("passed"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
