"""线路 B 离线验收：标准平贴姿 → GETUP → 稳定站立。"""

from __future__ import annotations

import json
from pathlib import Path

from apps import ref
from apps.teleop.gamepad import Edges, ScriptPad
from apps.teleop.machine import FOLD, FOLDED, GETUP, UNFOLD, WALK, Teleop
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]


def main() -> None:
    base = Package.load(PROJECT / "weights" / "dual_v3b")
    skill = Package.load(PROJECT / "weights" / "getup_vpd_ft")
    robot = MuJoCoBackend(base)
    tele = Teleop(
        base,
        skill,
        robot,
        ref.find("getup_facedown"),
        ref.find("liedown_facedown"),
    )
    pad = ScriptPad("idle:1,getup:12,idle:3", base.control_dt)
    edges = Edges()
    seen: set[str] = set()
    standing_upright: list[float] = []
    standing_height: list[float] = []

    while not pad.done:
        p = pad.read()
        tele.tick(p, edges(p))
        seen.add(tele.mode)
        if tele.mode == WALK:
            standing_upright.append(robot.upright())
            standing_height.append(robot.pelvis_height())

    forbidden = sorted(seen.intersection({FOLDED, UNFOLD, FOLD}))
    result = {
        "route": "manual_standard_prone -> GETUP -> WALK",
        "seen_modes": sorted(seen),
        "forbidden_storage_modes": forbidden,
        "final_mode": tele.mode,
        "min_standing_upright": round(min(standing_upright, default=-1.0), 4),
        "min_standing_height_m": round(min(standing_height, default=-1.0), 4),
        "final_upright": round(robot.upright(), 4),
        "final_height_m": round(robot.pelvis_height(), 4),
    }
    result["passed"] = (
        GETUP in seen
        and tele.mode == WALK
        and not forbidden
        and result["min_standing_upright"] > 0.75
        and result["min_standing_height_m"] > 0.35
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
