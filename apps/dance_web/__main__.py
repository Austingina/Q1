"""手机网页控制 Q1 的行走 / 舞蹈 / 收储状态机。

默认只运行 MuJoCo 仿真。真机必须同时给 ``--backend real --enable-real``；首次联调再加
``--dry-run``，指令只发到假话题，电机不会接收。
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
from apps.teleop.command import FWD  # noqa: E402
from apps.dance_web.control import WebControl  # noqa: E402
from apps.dance_web.server import open_server  # noqa: E402
from apps.teleop.machine import Teleop  # noqa: E402
from apps.teleop.voice import open_voice  # noqa: E402
from runtime import MuJoCoBackend, Package  # noqa: E402

DEFAULT_BASE = PROJECT / "weights" / "dual_v3b"
DEFAULT_SKILL = PROJECT / "weights" / "getup_vpd_ft"
DEFAULT_DANCE = PROJECT / "weights" / "q1_smooth_dance"
DRY_TOPIC = "/kabutack/dry_run/joint/command"


def package(path: Path, label: str) -> Package:
    if not (path / "deploy.yaml").exists():
        raise SystemExit(f"{label}权重包不存在：{path}")
    return Package.load(path)


def optional_skill(path: Path | None, motion: Path | None, label: str, arg: str) -> tuple[Package | None, Path | None]:
    if path is None:
        if motion is not None:
            raise SystemExit(f"--{arg}-ref 必须与 --{arg}-skill 一起使用")
        return None, None
    ref_path = motion or path / "motion.npz"
    skill = package(path, label)
    if not (path / "policy.onnx").exists() or not ref_path.exists():
        raise SystemExit(f"{label}技能包不完整：{path}（需要 policy.onnx 和 {ref_path}）")
    return skill, ref_path


def status_of(tele: Teleop) -> dict:
    rows = dict(tele.status())
    out = {
        "mode": tele.mode,
        "message": tele.msg,
        "vx": float(tele.cmd[0]),
        "vy": float(tele.cmd[1]),
        "wz": float(tele.cmd[2]),
        "upright": float(tele.robot.upright()),
        "can_unfold": bool(tele.has_unfold),
        "can_fold": bool(tele.has_fold),
    }
    if tele.mode == "DANCE":
        out["dance_seconds"] = round(tele.i * tele.dt, 2)
    if "t" in rows:
        out["progress"] = rows["t"]
    return out


def main() -> None:
    p = argparse.ArgumentParser(description="Q1 行走 / 舞蹈 / 收储手机网页")
    p.add_argument("--backend", choices=("sim", "real"), default="sim")
    p.add_argument("--enable-real", action="store_true", help="真机二次确认；--backend real 时必须给出")
    p.add_argument("--dry-run", action="store_true", help=f"真机演练：只发到 {DRY_TOPIC}")
    p.add_argument("--package", type=Path, default=DEFAULT_BASE)
    p.add_argument("--skill", type=Path, default=DEFAULT_SKILL)
    p.add_argument("--dance", type=Path, default=DEFAULT_DANCE)
    p.add_argument("--getup-ref", default="getup_facedown")
    p.add_argument("--liedown-ref", default="liedown_facedown")
    p.add_argument("--unfold-skill", type=Path, default=None, help="UNFOLD：深度折叠→平趴策略包")
    p.add_argument("--unfold-ref", type=Path, default=None, help="UNFOLD 参考 motion.npz")
    p.add_argument("--fold-skill", type=Path, default=None, help="FOLD：平趴→深度折叠策略包")
    p.add_argument("--fold-ref", type=Path, default=None, help="FOLD 参考 motion.npz")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8767)
    p.add_argument("--token", default=None, help="网页控制令牌；缺省每次启动随机生成")
    p.add_argument("--drive-timeout", type=float, default=0.5)
    p.add_argument("--safety-profile", choices=("off", "getup", "walk", "full"), default="off",
                   help="断联阻尼的真机安全配置：getup 只起身，walk 只前后低速，full 全功能")
    p.add_argument("--heartbeat-timeout", type=float, default=0.3, help="网页控制心跳超时后切阻尼")
    p.add_argument("--speed-scale", type=float, default=0.5)
    p.add_argument("--sensor-timeout", type=float, default=10.0)
    p.add_argument("--sim-start-standing", action="store_true", help="仿真直接从站姿开始，跳过起身")
    p.add_argument("--sim-start-folded", action="store_true", help="仿真从深度折叠开始，验证 UNFOLD→GETUP")
    a = p.parse_args()

    if a.backend == "real" and not a.enable_real:
        raise SystemExit("真机入口默认关闭；确认吊带、急停与低层模式后显式加 --enable-real")
    if a.backend == "sim" and a.dry_run:
        raise SystemExit("--dry-run 只用于 --backend real")
    if a.backend == "real" and not a.dry_run and a.safety_profile == "off":
        raise SystemExit("真机网页控制必须选择 --safety-profile getup/walk/full，不允许关闭断联阻尼")
    if a.safety_profile == "walk":
        a.speed_scale = min(a.speed_scale, 0.08 / FWD)  # 首测前进上限 0.08 m/s
    if a.sim_start_standing and a.sim_start_folded:
        raise SystemExit("--sim-start-standing 和 --sim-start-folded 不能同时使用")
    if a.sim_start_folded and a.backend != "sim":
        raise SystemExit("--sim-start-folded 只用于仿真")

    base, skill, dance = package(a.package, "基础"), package(a.skill, "起身"), package(a.dance, "舞蹈")
    dance_ref = a.dance / "motion.npz"
    if not dance_ref.exists():
        raise SystemExit(f"舞蹈参考不存在：{dance_ref}")
    getup_ref, liedown_ref = ref.find(a.getup_ref), ref.find(a.liedown_ref)
    unfold_skill, unfold_ref = optional_skill(a.unfold_skill, a.unfold_ref, "UNFOLD 展开", "unfold")
    fold_skill, fold_ref = optional_skill(a.fold_skill, a.fold_ref, "FOLD 折叠", "fold")
    voice = open_voice("off")

    if a.backend == "real":
        from runtime import PrimeBotBackend
        from runtime.primebot_io import vendor_mc_running

        if vendor_mc_running():
            if not a.dry_run:
                raise SystemExit("厂商运控 mc_app_main/mc_main 正在运行；不能并发启动低层控制器")
            print(f"[dance_web] 注意：厂商 MC 正在运行；dry-run 只发布到假话题 {DRY_TOPIC}，不会写电机话题。", flush=True)
        robot = PrimeBotBackend(base, **({"cmd_topic": DRY_TOPIC, "ramp_track_lim": float("inf")} if a.dry_run else {}))
        robot.wait_sensors(a.sensor_timeout)
    else:
        robot = MuJoCoBackend(base)

    tele = Teleop(base, skill, robot, getup_ref, liedown_ref, speed_scale=a.speed_scale,
                  voice=voice, dance=dance, dance_ref=dance_ref,
                  unfold_skill=unfold_skill, unfold_ref=unfold_ref,
                  fold_skill=fold_skill, fold_ref=fold_ref)
    if a.backend == "sim" and a.sim_start_standing:
        tele.place_standing()
    elif a.sim_start_folded:
        tele.place_folded()

    token = a.token or secrets.token_urlsafe(9)
    control = WebControl(a.drive_timeout, a.safety_profile, a.heartbeat_timeout)
    server = open_server(control, a.host, a.port, token)
    shown_host = "<本机或机器人IP>" if a.host in ("0.0.0.0", "::") else a.host
    print(f"[dance_web] 打开 http://{shown_host}:{server.server_port}/?token={token}", flush=True)
    print("[dance_web] 正常停止会减速归零；阻尼是独立处置。网页不是独立硬件急停。", flush=True)
    if control.safety_enabled:
        print(f"[dance_web] 安全配置 {a.safety_profile}：网页断开或 {a.heartbeat_timeout:.2f}s 无心跳即切阻尼。", flush=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    t_next = time.perf_counter()
    try:
        while True:
            pad, edges = control.sample()
            tele.tick(pad, edges)
            control.set_status(status_of(tele))
            if a.backend == "sim":
                t_next += tele.dt
                time.sleep(max(0.0, t_next - time.perf_counter()))
    except KeyboardInterrupt:
        print("[dance_web] 退出，切阻尼", flush=True)
    finally:
        server.shutdown()
        if a.backend == "real":
            robot.close()
        else:
            robot.damp()
        voice.close()


if __name__ == "__main__":
    main()
