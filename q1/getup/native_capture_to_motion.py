"""把 ``capture_native_folded_getup`` 记录转换成 SuperMimic Q1 motion.npz。

转换只接受原生里程计有效的成功记录；不会凭关节角猜测骨盆高度。输出可直接作为
``scripts/train/getup.sh`` 的单段训练参考。需要在 SuperMimic-Q1 训练环境运行。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[2]
SUPERMIMIC_SRC = PROJECT / "infra" / "SuperMimic-Q1" / "src"
sys.path.insert(0, str(SUPERMIMIC_SRC))


def nearest_resample(t: np.ndarray, values: np.ndarray, fps: float) -> tuple[np.ndarray, np.ndarray]:
    """按时间戳取最近样本；原始记录默认 200 Hz，目标为 50 Hz。"""
    t = np.asarray(t, dtype=np.float64)
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("采集时间戳无效或不严格递增")
    grid = np.arange(0.0, t[-1] + 0.5 / fps, 1.0 / fps)
    right = np.searchsorted(t, grid, side="left")
    right = np.clip(right, 0, len(t) - 1)
    left = np.maximum(right - 1, 0)
    choose_right = np.abs(t[right] - grid) <= np.abs(t[left] - grid)
    idx = np.where(choose_right, right, left)
    return grid, np.asarray(values)[idx]


def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).copy()
    if not np.isfinite(q).all():
        raise ValueError("根姿态四元数含 NaN/Inf")
    n = np.linalg.norm(q, axis=1, keepdims=True)
    if np.any(n < 0.5):
        raise ValueError("根姿态四元数范数异常")
    q /= n
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] *= -1
    return q


def load_joint_order(package: Path) -> list[str]:
    spec = yaml.safe_load((package / "deploy.yaml").read_text())
    return [str(name) for name in spec["joint_names"]]


def build_qpos(capture: Path, package: Path, fps: float) -> tuple[np.ndarray, dict]:
    d = np.load(capture)
    if "success" not in d or not bool(np.asarray(d["success"]).reshape(-1)[0]):
        raise ValueError("采集记录未标记为成功，不能用于训练")
    names = [str(name) for name in d["joint_names"]]
    target_names = load_joint_order(package)
    index = {name: i for i, name in enumerate(names)}
    missing = [name for name in target_names if name not in index]
    if missing:
        raise ValueError(f"采集缺少关节：{missing}")
    q = np.asarray(d["q"], dtype=np.float64)[:, [index[name] for name in target_names]]
    t = np.asarray(d["t_monotonic"], dtype=np.float64)
    pos = np.asarray(d["odom_pos"], dtype=np.float64)
    quat = np.asarray(d["odom_quat"], dtype=np.float64)
    valid = np.isfinite(pos).all(axis=1) & np.isfinite(quat).all(axis=1)
    coverage = float(valid.mean())
    if coverage < 0.95:
        raise ValueError(f"/odom 有效覆盖率只有 {coverage:.1%}；不猜测根轨迹，请重新采集")
    if not 0.02 <= float(np.nanmedian(pos[:, 2])) <= 1.5:
        raise ValueError(f"/odom 高度不合理（中位 {np.nanmedian(pos[:, 2]):.3f} m）")

    grid, q = nearest_resample(t, q, fps)
    _, pos = nearest_resample(t, pos, fps)
    _, quat = nearest_resample(t, quat, fps)
    quat = normalize_quat(quat)
    pos[:, :2] -= pos[0, :2]
    qpos = np.concatenate([pos, quat, q], axis=1)
    meta = {
        "frames": len(grid),
        "duration_s": float(grid[-1]),
        "fps": fps,
        "odom_coverage": coverage,
        "joint_names": target_names,
    }
    return qpos, meta


def main() -> None:
    p = argparse.ArgumentParser(description="原生折叠起身采集 → SuperMimic motion.npz")
    p.add_argument("capture", type=Path)
    p.add_argument("--package", type=Path, default=PROJECT / "weights" / "dual_v3b")
    p.add_argument("--output", type=Path, default=PROJECT / "apps" / "ref" / "skills" / "getup_folded_native.npz")
    p.add_argument("--fps", type=float, default=50.0)
    a = p.parse_args()
    if not 20.0 <= a.fps <= 100.0:
        p.error("--fps 必须在 20–100 Hz")
    qpos, meta = build_qpos(a.capture, a.package, a.fps)
    os.environ.setdefault("SUPERMIMIC_ROBOT", "q1")
    try:
        from supermimic.data.motion_convert import convert_qpos_to_npz
    except ImportError as error:
        raise SystemExit("缺少 SuperMimic-Q1 训练依赖；请在 infra/SuperMimic-Q1/.venv 中运行") from error
    summary = convert_qpos_to_npz(
        qpos,
        a.output,
        input_fps=a.fps,
        output_fps=a.fps,
        source=str(a.capture),
    )
    print(f"[folded-convert] {meta}")
    print(f"[folded-convert] {summary}")


if __name__ == "__main__":
    main()
