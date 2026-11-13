"""线路 B 事故后安全闭环：平趴起身、技能末帧保持、再平趴。"""

from __future__ import annotations

import json
from pathlib import Path

from apps import ref
from apps.line_b_web.machine import GETUP, LIEDOWN, LYING, STAND_HOLD, LineBMachine
from apps.teleop.command import FWD
from apps.teleop.gamepad import Edges, ScriptPad
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]


def main() -> None:
    base = Package.load(PROJECT / "weights/dual_v3b")
    skill = Package.load(PROJECT / "weights/getup_vpd_ft")
    dance = Package.load(PROJECT / "weights/q1_smooth_dance")
    robot = MuJoCoBackend(base)
    machine = LineBMachine(base, skill, robot, ref.find("getup_facedown"), ref.find("liedown_facedown"),
                           dance, dance.root / "motion.npz", speed_scale=0.08 / FWD)
    pad = ScriptPad("idle:1,getup:12,idle:5,liedown:12,idle:1", base.control_dt)
    edges = Edges()
    seen: set[str] = set()
    min_hold_upright = 1.0
    while not pad.done:
        p = pad.read()
        machine.tick(p, edges(p))
        seen.add(machine.mode)
        if machine.mode == STAND_HOLD:
            min_hold_upright = min(min_hold_upright, robot.upright())
    result = {
        "route": "B",
        "seen_modes": sorted(seen),
        "final_mode": machine.mode,
        "min_hold_upright": round(min_hold_upright, 4),
        "final_upright": round(robot.upright(), 4),
        "final_height_m": round(robot.pelvis_height(), 4),
    }
    result["passed"] = (
        {GETUP, STAND_HOLD, LIEDOWN}.issubset(seen)
        and machine.mode == LYING
        and min_hold_upright > 0.75
        and robot.upright() < 0.5
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
