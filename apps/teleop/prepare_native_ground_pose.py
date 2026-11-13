"""用原厂高层运控把 Q1 从折叠姿态展开到可接管的地面姿态。

该工具不发布低层关节指令，也不执行 ``STAND_UP``。它只在基础开发模式中
调用公开 SDK 动作 ``BIPED_GROUND_POSE``：原厂恢复器先把任意/折叠姿态
展开为稳定的仰卧或俯卧地面姿态。人工确认完全停稳后，才能在 APP
切回低层运控开发并重启，再使用自有 GETUP 策略起身。

默认只做服务和状态预检；必须显式加 ``--execute-native-action`` 并输入
``GROUND`` 才会调用动作。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))

from apps.teleop.capture_native_folded_getup import (  # noqa: E402
    GET_ACTION,
    SET_ACTION,
    get_action,
    set_action,
    wait_future,
    wait_service,
)
from runtime.primebot_io import vendor_mc_running  # noqa: E402

GET_AVAILABLE = "/aimdk_5Fmsgs/srv/GetMcAvailableActions"
GROUND_POSE = "BIPED_GROUND_POSE"
SAFE_START_ACTIONS = {"PASSIVE_DEFAULT", "LYING_DEFAULT", GROUND_POSE}


def get_available(node, client) -> set[str]:
    from aimdk_msgs.msg import CommonRequest
    from aimdk_msgs.srv import GetMcAvailableActions

    request = GetMcAvailableActions.Request()
    request.request = CommonRequest()
    request.request.header.stamp = node.get_clock().now().to_msg()
    response = wait_future(node, client.call_async(request), 3.0)
    if response is None or response.header.code != 0:
        raise RuntimeError("GetMcAvailableActions 返回失败")
    return {str(action) for action in response.actions}


def main() -> None:
    p = argparse.ArgumentParser(description="原厂折叠姿态→地面姿态（不起身）")
    p.add_argument(
        "--execute-native-action",
        action="store_true",
        help="明确允许调用一次 BIPED_GROUND_POSE；不带此参数只做预检",
    )
    p.add_argument("--yes", action="store_true", help="跳过键盘确认（仅用于已有独立现场确认流程）")
    p.add_argument("--observe-s", type=float, default=15.0, help="发送后监视 Action 状态的时间")
    a = p.parse_args()
    if not 5.0 <= a.observe_s <= 60.0:
        p.error("--observe-s 必须在 5–60 秒")
    if not vendor_mc_running():
        raise SystemExit("原厂高层运控未运行：请在 APP 退出低层运控开发模式并重启")

    import rclpy
    from aimdk_msgs.srv import GetMcAction, GetMcAvailableActions, SetMcAction

    rclpy.init()
    node = rclpy.create_node("locoinfra_prepare_native_ground_pose")
    get_client = node.create_client(GetMcAction, GET_ACTION)
    set_client = node.create_client(SetMcAction, SET_ACTION)
    available_client = node.create_client(GetMcAvailableActions, GET_AVAILABLE)
    try:
        wait_service(get_client, GET_ACTION, 8.0)
        wait_service(set_client, SET_ACTION, 8.0)
        wait_service(available_client, GET_AVAILABLE, 8.0)
        action, status = get_action(node, get_client)
        available = get_available(node, available_client)
        if GROUND_POSE not in available:
            raise RuntimeError(f"当前固件/状态未开放 {GROUND_POSE}")
        if action not in SAFE_START_ACTIONS:
            raise RuntimeError(f"当前 Action={action}，不允许从该状态展开折叠姿态")
        print(
            f"[ground-pose] 预检通过：Action={action} status={status}，{GROUND_POSE} 可用",
            flush=True,
        )
        if not a.execute_native_action:
            print("[ground-pose] 只做预检，未触发任何动作", flush=True)
            return
        if action == GROUND_POSE and status == 100:
            print("[ground-pose] 已在地面姿态控制中，不重复发送", flush=True)
            return
        if not a.yes:
            answer = input(
                "确认吊带/独立急停就绪、线缆移开、周围 1 m 无人；"
                "输入 GROUND 只展开到地面姿态："
            ).strip()
            if answer != "GROUND":
                print("[ground-pose] 已取消，未触发动作", flush=True)
                return

        print(f"[ground-pose] 调用原厂 {GROUND_POSE}；请勿靠近机器人", flush=True)
        set_action(node, set_client, GROUND_POSE)
        deadline = time.monotonic() + a.observe_s
        last: tuple[str, int] | None = None
        reached = False
        while rclpy.ok() and time.monotonic() < deadline:
            current = get_action(node, get_client)
            if current != last:
                print(f"[ground-pose] Action={current[0]} status={current[1]}", flush=True)
                last = current
            reached |= current[0] == GROUND_POSE and current[1] == 100
            time.sleep(0.2)
        if not reached:
            raise RuntimeError(f"未观测到 {GROUND_POSE} RUNNING，不得切换到低层模式")
        print(
            "[ground-pose] 原厂地面姿态已运行。必须由现场人员确认机器人已展开、"
            "完全停稳，再切回低层运控开发。",
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
