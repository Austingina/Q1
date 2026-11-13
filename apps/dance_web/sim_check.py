"""离线验收：起身 → 行走 → 停稳 → 舞蹈 → 回到站立。"""

from __future__ import annotations

import json
from pathlib import Path

from apps import ref
from apps.teleop.gamepad import Edges, ScriptPad
from apps.teleop.machine import DANCE, WALK, Teleop
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]


def main() -> None:
    base = Package.load(PROJECT / "weights/dual_v3b")
    skill = Package.load(PROJECT / "weights/getup_vpd_ft")
    dance = Package.load(PROJECT / "weights/q1_smooth_dance")
    robot = MuJoCoBackend(base)
    tele = Teleop(base, skill, robot, ref.find("getup_facedown"), ref.find("liedown_facedown"),
                  dance=dance, dance_ref=dance.root / "motion.npz")
    pad = ScriptPad("getup:12,fwd:3,idle:2,dance:24,idle:3", base.control_dt)
    edges = Edges()
    seen, min_h, min_up = set(), 99.0, 1.0
    while not pad.done:
        p = pad.read()
        tele.tick(p, edges(p))
        seen.add(tele.mode)
        if WALK in seen:
            min_h = min(min_h, robot.pelvis_height())
            min_up = min(min_up, robot.upright())
    result = {
        "seen_modes": sorted(seen),
        "final_mode": tele.mode,
        "min_height_after_getup": round(min_h, 4),
        "min_upright_after_getup": round(min_up, 4),
        "final_height": round(robot.pelvis_height(), 4),
        "final_upright": round(robot.upright(), 4),
    }
    result["passed"] = DANCE in seen and tele.mode == WALK and min_h > 0.2 and min_up > 0.3
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
