"""用公开 SDK 记录 Q1 原生“折叠趴卧 → 站立”动作。

这个程序不发布低层关节指令。它只能在原厂高层运控在线的基础开发模式运行，
通过 ``SetMcAction(STAND_UP)`` 触发一次原生恢复，同时记录关节、IMU 与可用的
里程计。记录可作为自有 ``UNFOLD`` 启动技能的补充训练数据，不复制厂商模型。

安全流程：
  1. APP 退出“低层运控开发”，进入保留原厂运控的开发模式并重启；
  2. 机器人保持折叠趴卧，充电线/网线移开，吊带和独立急停就绪；
  3. 先不带 ``--execute-native-action`` 验证传感器与服务；
  4. 人工确认后带该参数记录一次原生起身。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from apps.teleop.lying_pose import is_folded_prone  # noqa: E402
from runtime.primebot_io import vendor_mc_running  # noqa: E402

GET_ACTION = "/aimdk_5Fmsgs/srv/GetMcAction"
SET_ACTION = "/aimdk_5Fmsgs/srv/SetMcAction"
STAND_UP = "STAND_UP"
STANDING_ACTION = "BIPED_WALK_RUN"


def _qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _nan(n: int) -> np.ndarray:
    return np.full(n, np.nan, dtype=np.float64)


class Capture:
    def __init__(self, node, sample_hz: float):
        from aimdk_msgs.msg import JointStateArray
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu

        self.node = node
        self.period = 1.0 / sample_hz
        self.recording = False
        self.joint_names: list[str] | None = None
        self.latest_q: np.ndarray | None = None
        self.latest_qd: np.ndarray | None = None
        self.latest_effort: np.ndarray | None = None
        self.imu_quat = _nan(4)
        self.imu_gyro = _nan(3)
        self.odom_pos = _nan(3)
        self.odom_quat = _nan(4)
        self.odom_lin = _nan(3)
        self.odom_ang = _nan(3)
        self.torso_pos = _nan(3)
        self.torso_quat = _nan(4)
        self.action_desc = ""
        self.action_status = 0
        self._next_sample = 0.0
        self.rows: dict[str, list] = {k: [] for k in (
            "t_monotonic", "t_ros", "q", "qd", "effort", "imu_quat", "imu_gyro",
            "odom_pos", "odom_quat", "odom_lin", "odom_ang", "torso_pos", "torso_quat",
            "action_desc", "action_status",
        )}
        qos = _qos()
        node.create_subscription(JointStateArray, "/aima/hal/joint/state", self.on_joint, qos)
        node.create_subscription(Imu, "/aima/hal/imu/state", self.on_imu, qos)
        node.create_subscription(Odometry, "/odom", self.on_odom, qos)
        node.create_subscription(Odometry, "/odom_torso", self.on_torso, qos)

    def on_imu(self, msg) -> None:
        o, w = msg.orientation, msg.angular_velocity
        self.imu_quat = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
        self.imu_gyro = np.array([w.x, w.y, w.z], dtype=np.float64)

    @staticmethod
    def _odom(msg) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        return (
            np.array([p.x, p.y, p.z], dtype=np.float64),
            np.array([o.w, o.x, o.y, o.z], dtype=np.float64),
            np.array([v.x, v.y, v.z], dtype=np.float64),
            np.array([w.x, w.y, w.z], dtype=np.float64),
        )

    def on_odom(self, msg) -> None:
        self.odom_pos, self.odom_quat, self.odom_lin, self.odom_ang = self._odom(msg)

    def on_torso(self, msg) -> None:
        self.torso_pos, self.torso_quat, _, _ = self._odom(msg)

    def on_joint(self, msg) -> None:
        joints = msg.joints
        if self.joint_names is None:
            self.joint_names = [j.name for j in joints]
        by_name = {j.name: j for j in joints}
        if any(name not in by_name for name in self.joint_names):
            return
        ordered = [by_name[name] for name in self.joint_names]
        self.latest_q = np.array([j.position for j in ordered], dtype=np.float64)
        self.latest_qd = np.array([j.velocity for j in ordered], dtype=np.float64)
        self.latest_effort = np.array([j.effort for j in ordered], dtype=np.float64)
        now = time.monotonic()
        if not self.recording or now < self._next_sample:
            return
        self._next_sample = now + self.period
        stamp = msg.header.stamp
        values = {
            "t_monotonic": now,
            "t_ros": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            "q": self.latest_q.copy(),
            "qd": self.latest_qd.copy(),
            "effort": self.latest_effort.copy(),
            "imu_quat": self.imu_quat.copy(),
            "imu_gyro": self.imu_gyro.copy(),
            "odom_pos": self.odom_pos.copy(),
            "odom_quat": self.odom_quat.copy(),
            "odom_lin": self.odom_lin.copy(),
            "odom_ang": self.odom_ang.copy(),
            "torso_pos": self.torso_pos.copy(),
            "torso_quat": self.torso_quat.copy(),
            "action_desc": self.action_desc,
            "action_status": self.action_status,
        }
        for key, value in values.items():
            self.rows[key].append(value)

    def ready(self) -> bool:
        return self.latest_q is not None and np.isfinite(self.imu_quat).all()

    def save(self, path: Path, **meta) -> None:
        if not self.rows["t_monotonic"]:
            raise RuntimeError("没有采集到关节数据")
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {key: np.asarray(value) for key, value in self.rows.items()}
        data["t_monotonic"] -= data["t_monotonic"][0]
        np.savez_compressed(
            path,
            **data,
            joint_names=np.asarray(self.joint_names),
            source=np.asarray("PrimeBot public SDK SetMcAction(STAND_UP)"),
            sample_hz=np.asarray([1.0 / self.period], dtype=np.float32),
            **{key: np.asarray(value) for key, value in meta.items()},
        )
        print(f"[folded-capture] 已保存 {len(data['q'])} 帧 → {path}", flush=True)


def spin_for(node, seconds: float) -> None:
    import rclpy

    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


def wait_service(client, name: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            return
    raise RuntimeError(f"{name} 不可用：当前很可能仍是低层运控开发模式")


def wait_future(node, future, timeout: float):
    import rclpy

    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("服务调用超时")
    return future.result()


def get_action(node, client) -> tuple[str, int]:
    from aimdk_msgs.msg import CommonRequest
    from aimdk_msgs.srv import GetMcAction

    request = GetMcAction.Request()
    request.request = CommonRequest()
    request.request.header.stamp = node.get_clock().now().to_msg()
    response = wait_future(node, client.call_async(request), 2.0)
    if response is None or response.header.code != 0:
        raise RuntimeError("GetMcAction 返回失败")
    return str(response.info.action_desc), int(response.info.status.value)


def set_action(node, client, action: str) -> None:
    from aimdk_msgs.msg import CommonState, McActionCommand, RequestHeader
    from aimdk_msgs.srv import SetMcAction

    request = SetMcAction.Request()
    request.header = RequestHeader()
    request.header.stamp = node.get_clock().now().to_msg()
    request.source = "locoinfra_folded_capture"
    request.command = McActionCommand()
    request.command.action_desc = action
    response = wait_future(node, client.call_async(request), 3.0)
    if response is None:
        raise RuntimeError(f"SetMcAction({action}) 无响应")
    state = int(response.response.state.value)
    if response.response.header.code != 0 or state not in (CommonState.SUCCESS, CommonState.RUNNING):
        raise RuntimeError(
            f"SetMcAction({action}) 被拒绝：code={response.response.header.code}, state={state}, "
            f"reason={response.response.state.reason}"
        )


def main() -> None:
    p = argparse.ArgumentParser(description="记录原生折叠趴卧起身，供自有低层策略训练")
    p.add_argument("--execute-native-action", action="store_true",
                   help="明确允许调用一次原生 STAND_UP；不带此参数只做服务/姿态预检")
    p.add_argument("--yes", action="store_true", help="跳过键盘确认（仅用于已有独立现场确认流程）")
    p.add_argument("--sample-hz", type=float, default=200.0)
    p.add_argument("--timeout", type=float, default=30.0)
    p.add_argument("--pre-roll", type=float, default=1.0)
    p.add_argument("--post-roll", type=float, default=1.5)
    p.add_argument("--output", type=Path, default=None)
    a = p.parse_args()
    if not 20.0 <= a.sample_hz <= 500.0:
        p.error("--sample-hz 必须在 20–500 Hz")

    if not vendor_mc_running():
        raise SystemExit("原厂高层运控未运行：请先在 APP 退出低层运控开发模式并重新上下电")

    import rclpy
    from aimdk_msgs.srv import GetMcAction, SetMcAction

    rclpy.init()
    node = rclpy.create_node("locoinfra_capture_native_folded_getup")
    capture = Capture(node, a.sample_hz)
    get_client = node.create_client(GetMcAction, GET_ACTION)
    set_client = node.create_client(SetMcAction, SET_ACTION)
    success = False
    initial_action = final_action = ""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = a.output or PROJECT / "q1" / "getup" / "data" / f"native_folded_getup_{stamp}.npz"
    try:
        wait_service(get_client, GET_ACTION, 8.0)
        wait_service(set_client, SET_ACTION, 8.0)
        deadline = time.monotonic() + 8.0
        while not capture.ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not capture.ready():
            raise RuntimeError("未收到关节或 IMU 状态")
        if not is_folded_prone(capture.joint_names or (), capture.latest_q):
            raise RuntimeError("当前不是折叠趴卧姿态，拒绝采集折叠起身")
        initial_action, status = get_action(node, get_client)
        capture.action_desc, capture.action_status = initial_action, status
        print(f"[folded-capture] 预检通过：折叠趴卧，当前 Action={initial_action} status={status}", flush=True)
        if not a.execute_native_action:
            print("[folded-capture] 只做预检，未触发任何动作；确认现场安全后加 --execute-native-action", flush=True)
            return
        if not a.yes:
            answer = input("确认吊带/独立急停就绪、线缆移开、周围 1 m 无人；输入 STAND 开始原生起身：").strip()
            if answer != "STAND":
                print("[folded-capture] 已取消，未触发动作", flush=True)
                return

        capture.recording = True
        spin_for(node, a.pre_roll)
        print("[folded-capture] 调用原生 STAND_UP 并记录；请勿靠近机器人", flush=True)
        set_action(node, set_client, STAND_UP)
        stable_since: float | None = None
        deadline = time.monotonic() + a.timeout
        next_poll = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now < next_poll:
                continue
            next_poll = now + 0.2
            try:
                final_action, status = get_action(node, get_client)
                capture.action_desc, capture.action_status = final_action, status
            except (RuntimeError, TimeoutError) as error:
                print(f"[folded-capture] 状态查询暂时失败：{error}", flush=True)
                continue
            if final_action == STANDING_ACTION and status == 400:  # McActionStatus.RUNNING
                stable_since = stable_since or now
                if now - stable_since >= 1.0:
                    success = True
                    break
            else:
                stable_since = None
        if not success:
            raise TimeoutError(f"{a.timeout:.1f}s 内未确认进入 {STANDING_ACTION}/RUNNING")
        spin_for(node, a.post_roll)
        print("[folded-capture] 原生起身完成，记录结束", flush=True)
    except KeyboardInterrupt:
        print("[folded-capture] Ctrl-C：停止记录；不额外改变原厂运控状态", flush=True)
    finally:
        if capture.recording and capture.rows["t_monotonic"]:
            capture.save(
                output,
                success=np.asarray(success),
                initial_action=np.asarray(initial_action),
                final_action=np.asarray(final_action),
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
