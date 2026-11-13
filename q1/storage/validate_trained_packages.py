"""Offline acceptance gate for the trained Q1 UNFOLD/FOLD packages.

Run after ``eval_report.py``, both exports, and both ``deploy_sim_check.py`` calls::

    infra/bin/q1py q1/storage/validate_trained_packages.py

This script never opens a robot backend and never publishes motor commands.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from apps.teleop.lying_pose import is_folded_prone
from runtime import Package
from runtime.body_ref import BodyRef
from runtime.policy import SkillModel


PROJECT = Path(__file__).resolve().parents[2]
HANDOFF_LIMIT_RAD = 0.5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate trained Q1 storage packages without hardware access")
    p.add_argument("--base", type=Path, default=PROJECT / "weights/dual_v3b")
    p.add_argument("--getup", type=Path, default=PROJECT / "weights/getup_vpd_ft")
    p.add_argument("--unfold", type=Path, default=PROJECT / "weights/q1_unfold")
    p.add_argument("--fold", type=Path, default=PROJECT / "weights/q1_fold")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    errors: list[str] = []
    checks: dict[str, object] = {}

    def require(ok: bool, message: str) -> None:
        if not ok:
            errors.append(message)

    for label, root in (("UNFOLD", args.unfold), ("FOLD", args.fold)):
        for name in ("policy.onnx", "deploy.yaml", "motion.npz", "deploy_motion.npz", "meta.json", "report.json", "deploy_sim_check.json"):
            require((root / name).is_file(), f"{label} missing {root / name}")

    if errors:
        raise SystemExit("\n".join(["PACKAGE VALIDATION: FAIL", *[f"- {e}" for e in errors]]))

    base = Package.load(args.base)
    getup = Package.load(args.getup)
    unfold = Package.load(args.unfold)
    fold = Package.load(args.fold)
    packages = {"GETUP": getup, "UNFOLD": unfold, "FOLD": fold}

    for label, pkg in packages.items():
        require(pkg.joint_names == base.joint_names, f"{label} joint order differs from base")
        require(np.allclose(pkg.default_joint_pos, base.default_joint_pos), f"{label} default pose differs from base")
        require(np.allclose(pkg.action_scale, base.action_scale), f"{label} action scale differs from base")
        require(np.isclose(pkg.control_dt, base.control_dt), f"{label} control period differs from base")

    unfold_ref = BodyRef(unfold, unfold.root / "motion.npz")
    fold_ref = BodyRef(fold, fold.root / "motion.npz")
    getup_ref = BodyRef(getup, PROJECT / "apps/ref/skills/getup_facedown.npz")
    liedown_ref = BodyRef(getup, PROJECT / "apps/ref/skills/liedown_facedown.npz")

    require(unfold_ref.T == 640, f"UNFOLD frame count is {unfold_ref.T}, expected 640")
    require(fold_ref.T == 640, f"FOLD frame count is {fold_ref.T}, expected 640")
    require(is_folded_prone(base.joint_names, unfold_ref.joint_pos[0]), "UNFOLD does not start folded")
    require(not is_folded_prone(base.joint_names, unfold_ref.joint_pos[-1]), "UNFOLD still ends folded")
    require(not is_folded_prone(base.joint_names, fold_ref.joint_pos[0]), "FOLD already starts folded")
    require(is_folded_prone(base.joint_names, fold_ref.joint_pos[-1]), "FOLD does not end folded")

    unfold_handoff = float(np.abs(unfold_ref.joint_pos[-1] - getup_ref.joint_pos[0]).max())
    fold_handoff = float(np.abs(fold_ref.joint_pos[0] - liedown_ref.joint_pos[-1]).max())
    checks["unfold_to_getup_max_joint_rad"] = unfold_handoff
    checks["liedown_to_fold_max_joint_rad"] = fold_handoff
    require(unfold_handoff <= HANDOFF_LIMIT_RAD, f"UNFOLD->GETUP handoff {unfold_handoff:.3f} rad exceeds {HANDOFF_LIMIT_RAD}")
    require(fold_handoff <= HANDOFF_LIMIT_RAD, f"LIEDOWN->FOLD handoff {fold_handoff:.3f} rad exceeds {HANDOFF_LIMIT_RAD}")

    # Load and execute each ONNX once through the exact deployment wrapper.
    for label, pkg, ref in (("UNFOLD", unfold, unfold_ref), ("FOLD", fold, fold_ref)):
        model = SkillModel(pkg)
        obs_dim = int(pkg.spec["policy"]["inputs"]["obs"][1])
        obs = np.zeros((1, obs_dim), dtype=np.float32)
        ref.start(ref.anchor_quat[0])
        goal = ref.goal(0, ref.anchor_quat[0])
        action, target = model.act(obs, goal)
        require(action.shape == (len(base.joint_names),), f"{label} action shape is {action.shape}")
        require(target.shape == (len(base.joint_names),), f"{label} target shape is {target.shape}")
        require(np.isfinite(action).all() and np.isfinite(target).all(), f"{label} ONNX produced non-finite output")
        checks[f"{label.lower()}_goal_dim"] = int(goal.size)

        meta = json.loads((pkg.root / "meta.json").read_text())
        report = json.loads((pkg.root / "report.json").read_text())
        sim = json.loads((pkg.root / "deploy_sim_check.json").read_text())
        evaluation = meta.get("evaluation") or {}
        require(evaluation.get("passed") is True, f"{label} meta does not record passed evaluation")
        require(report.get("passed") is True, f"{label} report.json is not PASS")
        require(sim.get("fell_at_s") is None, f"{label} deploy simulation fell at {sim.get('fell_at_s')} s")
        require(sim.get("steps") == sim.get("of"), f"{label} deploy simulation ended early: {sim.get('steps')}/{sim.get('of')}")
        checks[f"{label.lower()}_deploy_sim"] = sim

    result = {"passed": not errors, "checks": checks, "errors": errors}
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if errors:
        raise SystemExit(2)
    print("PACKAGE VALIDATION: PASS (offline only; real-robot gates are still required)")


if __name__ == "__main__":
    main()
