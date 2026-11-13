"""Q1 真机假话题演练：读真实传感器、跑 E_tele 推理，但只向假话题发指令。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from apps.teleop.command import fill_goal  # noqa: E402
from runtime import Package, PrimeBotBackend, Session, TeleGoal  # noqa: E402

DRY_TOPIC = "/kabutack/dry_run/joint/command"
REAL_TOPIC = "/aima/hal/joint/command"
REQUIRED_TOPICS = {
    "/aima/hal/joint/state": "aimdk_msgs/msg/JointStateArray",
    "/aima/hal/imu/state": "sensor_msgs/msg/Imu",
}


def wait_required_topics(timeout: float) -> None:
    """先用主进程查 ROS graph；标准模式话题不存在时，不启动实时收发子进程。"""
    import rclpy

    rclpy.init()
    node = rclpy.create_node("kabutack_dry_run_preflight")
    try:
        end = time.monotonic() + timeout
        missing = dict(REQUIRED_TOPICS)
        while missing and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            graph = dict(node.get_topic_names_and_types())
            missing = {
                topic: expected
                for topic, expected in REQUIRED_TOPICS.items()
                if expected not in graph.get(topic, []) or not node.get_publishers_info_by_topic(topic)
            }
        if missing:
            text = "、".join(f"{topic} ({kind})" for topic, kind in missing.items())
            raise SystemExit(f"底层状态话题不可用：{text}。请在 APP 选择‘低层运控开发’并按厂商流程重启后再试。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    p = argparse.ArgumentParser(description="Q1 LocoInfra 假话题 dry-run")
    p.add_argument("--package", type=Path, default=PROJECT / "weights" / "dual_v3b")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--sensor-timeout", type=float, default=10.0)
    a = p.parse_args()
    if a.seconds <= 0 or a.seconds > 30:
        raise SystemExit("--seconds 必须在 (0, 30] 内")
    if DRY_TOPIC == REAL_TOPIC:
        raise SystemExit("安全检查失败：假话题不能等于电机话题")

    wait_required_topics(a.sensor_timeout)
    pkg = Package.load(a.package)
    robot = PrimeBotBackend(
        pkg,
        cmd_topic=DRY_TOPIC,
        ramp_s=0.25,
        ramp_track_lim=float("inf"),
        track_lim=float("inf"),
    )
    try:
        robot.wait_sensors(a.sensor_timeout)
        # ONNX 首次建立 Session 可能超过策略看门狗的 0.5 s。先在后端 IDLE（未发布）时预加载，
        # 再进入假话题 HOLD，避免把正常初始化误判为策略断流。
        session = Session(pkg, robot)
        # 从当前角度开始，不向默认站姿插值。即使假话题被误订阅，首帧也不会跳变。
        robot.reset(robot.sense().joint_pos)
        session.begin()
        goal = TeleGoal(pkg)
        fill_goal(goal, np.zeros(6, dtype=np.float64))

        frames = max(1, int(round(a.seconds / pkg.control_dt)))
        n_joint0, n_pub0 = robot.rates()
        infer_ms = []
        started = time.perf_counter()
        for _ in range(frames):
            t0 = time.perf_counter()
            action, target = session.model.act_tele(session.obs(), goal)
            infer_ms.append((time.perf_counter() - t0) * 1000.0)
            session.apply(action, target)
        elapsed = time.perf_counter() - started
        n_joint1, n_pub1 = robot.rates()
        result = {
            "ok": True,
            "command_topic": DRY_TOPIC,
            "real_motor_topic_used": False,
            "elapsed_s": round(elapsed, 3),
            "policy_frames": frames,
            "policy_hz": round(frames / elapsed, 1),
            "joint_state_frames": n_joint1 - n_joint0,
            "fake_command_frames": n_pub1 - n_pub0,
            "fake_command_hz": round((n_pub1 - n_pub0) / elapsed, 1),
            "inference_ms_mean": round(float(np.mean(infer_ms)), 3),
            "inference_ms_p99": round(float(np.percentile(infer_ms, 99)), 3),
            "control_overruns": robot.overruns,
            "upright": round(robot.upright(), 4),
        }
        if result["fake_command_hz"] < 400.0:
            result["ok"] = False
            result["error"] = "假指令频率低于 400 Hz"
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        if not result["ok"]:
            raise SystemExit(2)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
