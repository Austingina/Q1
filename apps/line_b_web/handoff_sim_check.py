"""离线扫描线路 B 起身→基础行走策略的交接时长。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from apps import ref
from apps.line_b_web.machine import (
    DAMPED,
    DANCE,
    LIEDOWN,
    LYING,
    STAND_HOLD,
    WALK,
    HandoffConfig,
    LineBMachine,
)
from apps.teleop.command import FWD
from apps.teleop.gamepad import Pad
from runtime import MuJoCoBackend, Package

PROJECT = Path(__file__).resolve().parents[2]


def run_until(machine, mode: str, max_s: float) -> bool:
    for _ in range(int(round(max_s / machine.dt))):
        machine.tick(Pad(), set())
        if machine.mode == mode:
            return True
        if machine.mode == DAMPED:
            return False
    return False


def run_for(machine, seconds: float, pad: Pad | None = None) -> bool:
    pad = pad or Pad()
    for _ in range(int(round(seconds / machine.dt))):
        machine.tick(pad, set())
        if machine.mode == DAMPED:
            return False
    return True


def run_case(
    duration: float,
    *,
    hold_s: float = 5.0,
    joint_perturb_rad: float = 0.0,
    perturb_seed: int = 0,
) -> dict:
    base = Package.load(PROJECT / "weights/dual_v3b")
    skill = Package.load(PROJECT / "weights/getup_vpd_ft")
    dance = Package.load(PROJECT / "weights/q1_smooth_dance")
    robot = MuJoCoBackend(base)
    machine = LineBMachine(
        base,
        skill,
        robot,
        ref.find("getup_facedown"),
        ref.find("liedown_facedown"),
        dance,
        dance.root / "motion.npz",
        speed_scale=0.08 / FWD,
        experimental_handoff=HandoffConfig(duration_s=duration),
        dance_enabled=True,
    )
    machine.tick(Pad(), {"GETUP"})
    reached_hold = run_until(machine, STAND_HOLD, 12.0)
    if reached_hold:
        run_for(machine, hold_s)
        if joint_perturb_rad > 0.0:
            rng = np.random.default_rng(perturb_seed)
            delta = rng.uniform(-joint_perturb_rad, joint_perturb_rad, len(robot.q_adr))
            robot.data.qpos[robot.q_adr] += delta
            mujoco.mj_forward(robot.model, robot.data)
            # 让保持控制器先看到并吸收扰动，再开始策略交接。
            run_for(machine, 0.25)
        machine.tick(Pad(), {"WALK"})
    reached_walk = run_until(machine, WALK, duration + 5.0)
    min_walk_upright = 1.0
    seen = {machine.mode}
    if reached_walk:
        for _ in range(int(round(3.0 / machine.dt))):
            machine.tick(Pad(ly=1.0), set())
            seen.add(machine.mode)
            min_walk_upright = min(min_walk_upright, robot.upright())
        for _ in range(int(round(2.0 / machine.dt))):
            machine.tick(Pad(), set())
        machine.tick(Pad(), {"DANCE"})
        run_until(machine, DANCE, 3.0)
        seen.add(machine.mode)
        run_until(machine, WALK, 24.0)
        seen.add(machine.mode)
        for _ in range(int(round(2.0 / machine.dt))):
            machine.tick(Pad(ly=1.0), set())
            min_walk_upright = min(min_walk_upright, robot.upright())
        machine.tick(Pad(), {"LIEDOWN"})
        run_until(machine, LIEDOWN, 3.0)
        seen.add(machine.mode)
        run_until(machine, LYING, 12.0)
        seen.add(machine.mode)
    result = {
        "duration_s": duration,
        "hold_s": hold_s,
        "joint_perturb_rad": joint_perturb_rad,
        "perturb_seed": perturb_seed,
        "reached_hold": reached_hold,
        "reached_walk": reached_walk,
        "final_mode": machine.mode,
        "seen_after_handoff": sorted(seen),
        "min_walk_upright": round(min_walk_upright, 6),
        "handoff": machine.handoff_metrics.as_dict(),
    }
    h = result["handoff"]
    result["passed"] = bool(
        reached_hold
        and reached_walk
        and machine.mode == LYING
        and not h["failure"]
        and h["max_raw_target_step"] <= 0.010001
        and h["max_applied_target_step"] <= 0.010001
        and h["min_upright"] >= 0.85
        and min_walk_upright >= 0.75
    )
    return result


def main() -> None:
    p = argparse.ArgumentParser(description="线路 B 离线交接时长扫描")
    p.add_argument("--durations", default="1,3,5,7,9,10,12")
    a = p.parse_args()
    durations = [float(x) for x in a.durations.split(",")]
    results = [run_case(x) for x in durations]
    # 7 s 标准工况可通过，但扰动回归要到 10 s 才能 8/8 不依赖限幅通过。
    selected = next((r["duration_s"] for r in results if r["passed"] and r["duration_s"] >= 10.0), None)
    out = {"offline_only": True, "results": results, "selected_duration_s": selected}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if selected is None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
