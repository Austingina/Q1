"""用正式线路 B 状态机与网页输入仲裁做纯 MuJoCo 交接回归。"""

from __future__ import annotations

import json
import os
from pathlib import Path

from apps import ref
from apps.line_b_web.control import LineBControl
from apps.line_b_web.machine import (
    DAMPED,
    DANCE,
    LIEDOWN,
    LYING,
    STAND_HOLD,
    WALK,
    WALK_HANDOFF,
    WALK_SETTLE,
    HandoffConfig,
    LineBMachine,
)
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]
DANCE_PACKAGE = Path(os.environ.get("LINE_B_DANCE_PACKAGE", PROJECT / "weights/q1_smooth_dance"))


def build() -> tuple[LineBMachine, LineBControl]:
    base = Package.load(PROJECT / "weights/dual_v3b")
    skill = Package.load(PROJECT / "weights/getup_vpd_ft")
    dance = Package.load(DANCE_PACKAGE)
    robot = MuJoCoBackend(base)
    machine = LineBMachine(
        base,
        skill,
        robot,
        ref.find("getup_facedown"),
        ref.find("liedown_facedown"),
        dance,
        dance.root / "motion.npz",
        speed_scale=0.5,
        experimental_handoff=HandoffConfig(),
        dance_enabled=True,
    )
    return machine, LineBControl(drive_timeout=100.0, heartbeat_timeout=100.0,
                                 mobility_enabled=True, dance_enabled=True)


def run_until(machine: LineBMachine, control: LineBControl, target: str, max_s: float, seen: set[str]) -> bool:
    for _ in range(int(round(max_s / machine.dt))):
        pad, edges = control.sample()
        machine.tick(pad, edges)
        # 正式网页主循环每帧都会把状态机模式回写给输入仲裁器。
        # 验收脚本必须复现这一步，否则到达 WALK 后仍会拒绝非零输入。
        control.set_status({"mode": machine.mode})
        seen.add(machine.mode)
        if machine.mode == target:
            return True
        if machine.mode == DAMPED:
            return False
    return False


def full_web_flow() -> dict:
    machine, control = build()
    seen: set[str] = set()
    control.session(True)
    control.action("getup")
    reached_hold = run_until(machine, control, STAND_HOLD, 12.0, seen)
    control.action("walk")
    reached_walk = run_until(machine, control, WALK, machine.handoff.duration_s + machine.handoff.settle_s + 5.0, seen)
    if reached_walk:
        control.drive(ly=1.0)
        run_until(machine, control, "never", 2.0, seen)
        control.drive(ly=0.0)
        control.action("dance")
        run_until(machine, control, DANCE, 3.0, seen)
        dance_returned = run_until(machine, control, WALK, (machine.dance_ref.T - 1) * machine.dt + 5.0, seen)
        control.action("liedown")
        run_until(machine, control, LIEDOWN, 3.0, seen)
        reached_lying = run_until(machine, control, LYING, 12.0, seen)
    else:
        dance_returned = reached_lying = False
    required = {WALK_HANDOFF, WALK_SETTLE, WALK, DANCE, LIEDOWN, LYING}
    return {
        "reached_hold": reached_hold,
        "reached_walk": reached_walk,
        "dance_returned": dance_returned,
        "reached_lying": reached_lying,
        "seen_modes": sorted(seen),
        "handoff": machine.handoff_metrics.as_dict(),
        "passed": reached_hold and reached_walk and dance_returned and reached_lying and required.issubset(seen),
    }


def disconnect_during_handoff() -> dict:
    machine, control = build()
    seen: set[str] = set()
    control.session(True)
    control.action("getup")
    reached_hold = run_until(machine, control, STAND_HOLD, 12.0, seen)
    control.action("walk")
    reached_handoff = run_until(machine, control, WALK_HANDOFF, 1.0, seen)
    control.session(False)
    pad, edges = control.sample()
    machine.tick(pad, edges)
    return {
        "reached_hold": reached_hold,
        "reached_handoff": reached_handoff,
        "disconnect_edges": sorted(edges),
        "final_mode": machine.mode,
        # 当前真机规则：网页断联只将人工速度归零，不自动切阻尼。
        "passed": reached_hold and reached_handoff and not edges and machine.mode == WALK_HANDOFF,
    }


def main() -> None:
    locked = LineBControl()
    locked.session(True)
    try:
        locked.action("walk")
        default_locked = False
    except ValueError:
        default_locked = True
    full = full_web_flow()
    disconnect = disconnect_during_handoff()
    passed = default_locked and full["passed"] and disconnect["passed"]
    print(json.dumps({
        "offline_only": True,
        "default_locked": default_locked,
        "full_web_flow": full,
        "disconnect_during_handoff": disconnect,
        "passed": passed,
    }, ensure_ascii=False, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
