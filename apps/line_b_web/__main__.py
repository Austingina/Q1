"""线路 B 专用手机网页。

默认只运行 MuJoCo；真机必须同时给 ``--backend real --enable-real``。端口默认 8768，
与线路 A 的 8767 分开。
"""

from __future__ import annotations

import argparse
import secrets
import signal
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from apps import ref  # noqa: E402
from apps.line_b_web.control import LineBControl  # noqa: E402
from apps.line_b_web.machine import HandoffConfig, LineBMachine  # noqa: E402
from apps.line_b_web.server import open_server  # noqa: E402
from apps.teleop.command import ACC, BACK, FWD, TURN  # noqa: E402
from apps.teleop.voice import open_voice  # noqa: E402
from runtime import MuJoCoBackend, Package  # noqa: E402

DEFAULT_BASE = PROJECT / "weights" / "dual_v3b"
DEFAULT_SKILL = PROJECT / "weights" / "getup_vpd_ft"
DEFAULT_DANCE = PROJECT / "weights" / "q1_smooth_dance"
DRY_TOPIC = "/kabutack/dry_run/joint/command"
ORIGINAL_REAL_SPEED_SCALE = 0.5


def package(path: Path, label: str, needs_scene: bool = False) -> Package:
    required = ("deploy.yaml", "scene.xml") if needs_scene else ("deploy.yaml",)
    if any(not (path / name).exists() for name in required):
        raise SystemExit(f"{label}权重包不完整：{path}")
    return Package.load(path)


def status_of(machine: LineBMachine, speed_scale: float, backend: str) -> dict:
    rows = dict(machine.status())
    out = {
        "route": "B",
        "mode": machine.mode,
        "message": machine.msg,
        "vx": float(machine.cmd[0]),
        "wz": float(machine.cmd[2]),
        "upright": float(machine.robot.upright()),
        "speed_scale": float(speed_scale),
        "max_forward": float(speed_scale * FWD),
        "max_backward": float(speed_scale * BACK),
        "max_turn": float(speed_scale * TURN),
        "linear_accel": float(ACC[0]),
        "turn_accel": float(ACC[2]),
        "control_hz": float(1.0 / machine.dt),
        "dance_duration_s": float((machine.dance_ref.T - 1) * machine.dt),
        "backend": backend,
        "handoff_candidate_enabled": machine.handoff is not None,
        "pose_check": dict(machine.pose_check_result),
    }
    if "t" in rows:
        out["progress"] = rows["t"]
    if machine.handoff is not None:
        out["handoff"] = machine.handoff_metrics.as_dict()
    return out


def main() -> None:
    p = argparse.ArgumentParser(description="Q1 线路 B：摆姿检查 / 标准平趴起身 / 人工行走和转向 / 完整舞蹈 / 趴下")
    p.add_argument("--backend", choices=("sim", "real"), default="sim")
    p.add_argument("--enable-real", action="store_true", help="真机二次确认")
    p.add_argument("--dry-run", action="store_true", help=f"真机演练：命令只发到 {DRY_TOPIC}")
    p.add_argument("--package", type=Path, default=DEFAULT_BASE)
    p.add_argument("--skill", type=Path, default=DEFAULT_SKILL)
    p.add_argument("--dance", type=Path, default=DEFAULT_DANCE)
    p.add_argument("--getup-ref", default="getup_facedown")
    p.add_argument("--liedown-ref", default="liedown_facedown")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8768)
    p.add_argument("--token", default=None)
    p.add_argument("--drive-timeout", type=float, default=0.25)
    p.add_argument("--heartbeat-timeout", type=float, default=1.0)
    p.add_argument("--speed-scale", type=float, default=ORIGINAL_REAL_SPEED_SCALE,
                   help=("原仓库统一速度比例；真机示例和开机自启默认 0.5，"
                         "前进/后退/转向不再单独覆盖"))
    p.add_argument("--sensor-timeout", type=float, default=10.0)
    p.add_argument("--sim-start-standing", action="store_true")
    p.add_argument("--experimental-sim-handoff", action="store_true",
                   help="仅 MuJoCo：启用 10 s 起身→零速行走交接候选；真机禁止")
    p.add_argument("--enable-manual-drive", action="store_true",
                   help="显式启用线路 B 按住前进/后退和转向；舞蹈由 --enable-dance 独立控制")
    p.add_argument("--enable-dance", action="store_true",
                   help="显式启用线路 B 完整舞蹈；必须与 --enable-manual-drive 同时使用")
    a = p.parse_args()

    if a.backend == "real" and not a.enable_real:
        raise SystemExit("线路 B 真机入口默认关闭；确认吊带、独立急停和低层模式后加 --enable-real")
    if a.backend == "sim" and a.dry_run:
        raise SystemExit("--dry-run 只用于 --backend real")
    if a.experimental_sim_handoff and a.backend != "sim":
        raise SystemExit("--experimental-sim-handoff 只允许 MuJoCo，真机和真机演练均禁止")
    if a.experimental_sim_handoff and a.sim_start_standing:
        raise SystemExit("交接实验必须从平趴起身开始，不能与 --sim-start-standing 同用")
    if a.experimental_sim_handoff and a.enable_manual_drive:
        raise SystemExit("--experimental-sim-handoff 与 --enable-manual-drive 不能同时使用")
    if a.enable_dance and not a.enable_manual_drive:
        raise SystemExit("--enable-dance 必须与 --enable-manual-drive 同时使用，确保先完成零速交接进入 WALK")
    if not 0 < a.speed_scale <= ORIGINAL_REAL_SPEED_SCALE:
        raise SystemExit(f"--speed-scale 必须在 (0, {ORIGINAL_REAL_SPEED_SCALE:.1f}]；真机配置为 0.5")

    base = package(a.package, "基础", needs_scene=True)
    skill = package(a.skill, "起身 / 趴下")
    dance = package(a.dance, "舞蹈")
    dance_ref = a.dance / "motion.npz"
    if not dance_ref.exists():
        raise SystemExit(f"舞蹈参考不存在：{dance_ref}")

    voice = open_voice("off")
    if a.backend == "real":
        from runtime import PrimeBotBackend
        from runtime.primebot_io import vendor_mc_running

        if vendor_mc_running() and not a.dry_run:
            raise SystemExit("厂商运控 mc 正在运行；必须先切换到「低层运控开发」")
        kwargs = {"cmd_topic": DRY_TOPIC, "ramp_track_lim": float("inf")} if a.dry_run else {}
        robot = PrimeBotBackend(base, **kwargs)
        robot.wait_sensors(a.sensor_timeout)
    else:
        robot = MuJoCoBackend(base)

    mobility_enabled = a.experimental_sim_handoff or a.enable_manual_drive
    dance_enabled = a.experimental_sim_handoff or a.enable_dance
    machine = LineBMachine(
        base,
        skill,
        robot,
        ref.find(a.getup_ref),
        ref.find(a.liedown_ref),
        dance,
        dance_ref,
        speed_scale=a.speed_scale,
        voice=voice,
        experimental_handoff=HandoffConfig() if mobility_enabled else None,
        allow_real_manual_drive=a.backend == "real" and a.enable_manual_drive,
        dance_enabled=dance_enabled,
    )
    if a.backend == "sim" and a.sim_start_standing:
        machine.place_standing()

    token = a.token or secrets.token_urlsafe(9)
    control = LineBControl(a.drive_timeout, a.heartbeat_timeout,
                           mobility_enabled=mobility_enabled, dance_enabled=dance_enabled)
    server = open_server(control, a.host, a.port, token)
    shown_host = "<机器人IP>" if a.host in ("0.0.0.0", "::") else a.host
    print(f"[line-b] 打开 http://{shown_host}:{server.server_port}/?token={token}", flush=True)
    if a.enable_manual_drive and a.enable_dance:
        dance_seconds = (machine.dance_ref.T - 1) * machine.dt
        mode_note = (f"人工行走、转向和完整舞蹈已启用（原仓库 speed-scale={a.speed_scale:.1f}；"
                     f"前进 ≤ {a.speed_scale * FWD:.2f}、后退 ≤ {a.speed_scale * BACK:.2f} m/s、"
                     f"转向 ≤ {a.speed_scale * TURN:.2f} rad/s；"
                     f"舞蹈 {dance_seconds:.1f} s，只允许从 WALK 零速站稳后开始）")
    elif a.enable_manual_drive:
        mode_note = (f"人工行走和转向已启用（原仓库 speed-scale={a.speed_scale:.1f}；"
                     f"前进 ≤ {a.speed_scale * FWD:.2f}、后退 ≤ {a.speed_scale * BACK:.2f} m/s、"
                     f"转向 ≤ {a.speed_scale * TURN:.2f} rad/s）；"
                     "舞蹈未启用")
    elif a.experimental_sim_handoff:
        mode_note = "仅 MuJoCo 交接实验已启用"
    else:
        mode_note = "事故锁定：行走/舞蹈关闭"
    print(f"[line-b] 独立端口 {server.server_port}；{mode_note}；不含线路 A 折叠动作", flush=True)
    print(f"[line-b] 网页断开或 {a.heartbeat_timeout:.2f}s 无心跳即速度归零并保持当前控制状态；"
          "只有显式急停才切阻尼，网页不能代替独立硬件急停", flush=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    t_next = time.perf_counter()
    try:
        while True:
            pad, edges = control.sample()
            machine.tick(pad, edges)
            control.set_status(status_of(machine, a.speed_scale, a.backend))
            if a.backend == "sim":
                t_next += machine.dt
                time.sleep(max(0.0, t_next - time.perf_counter()))
    except KeyboardInterrupt:
        print("[line-b] 退出，切阻尼", flush=True)
    finally:
        server.shutdown()
        server.server_close()
        if a.backend == "real":
            robot.close()
        else:
            robot.damp()
        voice.close()


if __name__ == "__main__":
    main()
