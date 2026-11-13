"""Build the two Q1 storage-motion references used by the low-level controller.

The vendor crawl-to-fold reference ends in an intermediate floor pose, while the real
power-off pose is the deeper symmetric fold used by ``q1_default2fold_v2.csv``.  This
builder makes one continuous, kinematically consistent reference:

    existing LIEDOWN endpoint -> vendor crawl-to-fold -> smooth deep-fold bridge

and reverses it for startup.  MuJoCo forward kinematics is recomputed for every frame;
we never splice body poses from incompatible roots.  The result is training data, not a
real-robot command trajectory: policies must still be trained and pass simulation,
dry-run, harness and independent-estop gates before use on hardware.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_CRAWL = PROJECT / "q1/storage/source/q1_crawl2fold_v1.1_20260722.npz"
DEFAULT_DEEP = PROJECT / "q1/storage/source/q1_default2fold_v2.csv"
DEFAULT_LYING = PROJECT / "apps/ref/skills/liedown_facedown.npz"
DEFAULT_MODEL = PROJECT / "q1/q1_v3/xml/q1_v3_collision.xml"
DEFAULT_OUT = PROJECT / "q1/storage/motions"
FPS = 50.0


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _portable(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT.resolve()))
    except ValueError:
        return str(path)


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ),
        axis=-1,
    )


def _qconj(q: np.ndarray) -> np.ndarray:
    return q * np.array((1.0, -1.0, -1.0, -1.0))


def _qmat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        )
    )


def _slerp(a: np.ndarray, b: np.ndarray, u: np.ndarray) -> np.ndarray:
    a, b = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        q = (1 - u[:, None]) * a + u[:, None] * b
    else:
        theta = np.arccos(dot)
        q = np.sin((1 - u) * theta)[:, None] / np.sin(theta) * a
        q += np.sin(u * theta)[:, None] / np.sin(theta) * b
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _smooth(n: int) -> np.ndarray:
    x = np.linspace(0.0, 1.0, n)
    return 10 * x**3 - 15 * x**4 + 6 * x**5


def _bridge(a: np.ndarray, b: np.ndarray, n: int) -> np.ndarray:
    u = _smooth(n)[:, None]
    return (1 - u) * a + u * b


def _rotvec(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.where(q[..., :1] < 0, -q, q)
    v = q[..., 1:]
    s = np.linalg.norm(v, axis=-1)
    angle = 2 * np.arctan2(s, q[..., 0])
    scale = np.where(s > 1e-9, angle / np.where(s > 1e-9, s, 1.0), 2.0)
    return v * scale[..., None]


def _angular_velocity(quat: np.ndarray, dt: float) -> np.ndarray:
    out = np.zeros(quat.shape[:-1] + (3,), dtype=np.float64)
    if len(quat) < 3:
        return out
    rel = _qmul(quat[2:], _qconj(quat[:-2]))
    out[1:-1] = _rotvec(rel) / (2 * dt)
    out[0], out[-1] = out[1], out[-2]
    return out


def _forward_kinematics(
    model_path: Path,
    joint_names: list[str],
    body_names: list[str],
    root_pos: np.ndarray,
    root_quat: np.ndarray,
    joint_pos: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    free = next(j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
    fq = int(model.jnt_qposadr[free])
    qadr = []
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError(f"model is missing joint {name}")
        qadr.append(int(model.jnt_qposadr[jid]))
    bids = []
    for name in body_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise ValueError(f"model is missing body {name}")
        bids.append(bid)

    pos = np.empty((len(joint_pos), len(bids), 3), dtype=np.float64)
    quat = np.empty((len(joint_pos), len(bids), 4), dtype=np.float64)
    for i in range(len(joint_pos)):
        data.qpos[fq : fq + 3] = root_pos[i]
        data.qpos[fq + 3 : fq + 7] = root_quat[i]
        data.qpos[qadr] = joint_pos[i]
        mujoco.mj_forward(model, data)
        pos[i], quat[i] = data.xpos[bids], data.xquat[bids]
    return pos, quat


def _write_motion(
    path: Path,
    joint_names: list[str],
    body_names: list[str],
    root_pos: np.ndarray,
    root_quat: np.ndarray,
    joint_pos: np.ndarray,
    model_path: Path,
    source: dict,
) -> None:
    body_pos, body_quat = _forward_kinematics(
        model_path, joint_names, body_names, root_pos, root_quat, joint_pos
    )
    dt = 1.0 / FPS
    joint_vel = np.gradient(joint_pos, dt, axis=0)
    body_lin = np.gradient(body_pos, dt, axis=0)
    body_ang = np.stack([_angular_velocity(body_quat[:, b], dt) for b in range(len(body_names))], axis=1)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        fps=np.array([FPS], dtype=np.float32),
        joint_pos=joint_pos.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        body_pos_w=body_pos.astype(np.float32),
        body_quat_w=body_quat.astype(np.float32),
        body_lin_vel_w=body_lin.astype(np.float32),
        body_ang_vel_w=body_ang.astype(np.float32),
        joint_names=np.array(joint_names),
        body_names=np.array(body_names),
        source=np.array(json.dumps(source, ensure_ascii=False, sort_keys=True)),
        input_fps=np.array([FPS], dtype=np.float32),
    )


def build(crawl_path: Path, deep_path: Path, lying_path: Path, model_path: Path, out_dir: Path) -> dict:
    with np.load(crawl_path) as crawl, np.load(lying_path) as lying:
        cq = np.asarray(crawl["joint_pos"], dtype=np.float64)
        cp = np.asarray(crawl["body_pos_w"][:, 0], dtype=np.float64)
        cr = np.asarray(crawl["body_quat_w"][:, 0], dtype=np.float64)
        joint_names = [str(x) for x in lying["joint_names"]]
        body_names = [str(x) for x in lying["body_names"]]
        root = body_names.index("pelvis")
        lq = np.asarray(lying["joint_pos"][-1], dtype=np.float64)
        lp = np.asarray(lying["body_pos_w"][-1, root], dtype=np.float64)
        lr = np.asarray(lying["body_quat_w"][-1, root], dtype=np.float64)

    if cq.shape[1] != len(joint_names):
        raise ValueError(f"crawl joints {cq.shape[1]} != expected {len(joint_names)}")
    deep_all = np.loadtxt(deep_path, delimiter=",", dtype=np.float64)
    if deep_all.ndim != 2 or deep_all.shape[1] != len(joint_names):
        raise ValueError(f"deep-fold CSV has shape {deep_all.shape}")
    deep = deep_all[-1]

    # Match the vendor crawl reference to the existing face-down lie endpoint using a
    # gravity-preserving world-yaw rotation and translation.
    m = _qmat(lr) @ _qmat(cr[0]).T
    yaw = float(np.arctan2(m[1, 0] - m[0, 1], m[0, 0] + m[1, 1]))
    align = np.array((np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)))
    rz = _qmat(align)
    cp = lp + (rz @ (cp - cp[0]).T).T
    cr = _qmul(np.broadcast_to(align, cr.shape), cr)

    n_entry, n_deep, n_hold = int(FPS * 1.0) + 1, int(FPS * 3.0) + 1, int(FPS * 0.5)
    entry_q = _bridge(lq, cq[0], n_entry)
    entry_p = _bridge(lp, cp[0], n_entry)
    entry_r = _slerp(lr, cr[0], _smooth(n_entry))
    deep_q = _bridge(cq[-1], deep, n_deep)
    deep_p = np.repeat(cp[-1][None], n_deep, axis=0)
    deep_r = np.repeat(cr[-1][None], n_deep, axis=0)

    fold_q = np.concatenate((entry_q, cq[1:], deep_q[1:], np.repeat(deep[None], n_hold, axis=0)))
    fold_p = np.concatenate((entry_p, cp[1:], deep_p[1:], np.repeat(cp[-1][None], n_hold, axis=0)))
    fold_r = np.concatenate((entry_r, cr[1:], deep_r[1:], np.repeat(cr[-1][None], n_hold, axis=0)))
    source = {
        "builder": _portable(Path(__file__)),
        "crawl": {"file": _portable(crawl_path), "sha256": _sha256(crawl_path)},
        "deep_fold": {"file": _portable(deep_path), "sha256": _sha256(deep_path)},
        "lying": {"file": _portable(lying_path), "sha256": _sha256(lying_path)},
        "entry_bridge_s": 1.0,
        "deep_bridge_s": 3.0,
        "hold_s": 0.5,
    }
    fold_path, unfold_path = out_dir / "fold_facedown.npz", out_dir / "unfold_facedown.npz"
    _write_motion(fold_path, joint_names, body_names, fold_p, fold_r, fold_q, model_path, source | {"direction": "fold"})
    _write_motion(
        unfold_path,
        joint_names,
        body_names,
        fold_p[::-1].copy(),
        fold_r[::-1].copy(),
        fold_q[::-1].copy(),
        model_path,
        source | {"direction": "unfold"},
    )
    result = {
        "fold": _portable(fold_path),
        "unfold": _portable(unfold_path),
        "frames": int(len(fold_q)),
        "duration_s": (len(fold_q) - 1) / FPS,
        "entry_joint_max_rad": float(np.abs(lq - cq[0]).max()),
        "deep_bridge_joint_max_rad": float(np.abs(cq[-1] - deep).max()),
        "yaw_alignment_rad": yaw,
        "output_sha256": {"fold": _sha256(fold_path), "unfold": _sha256(unfold_path)},
        "source": source,
    }
    (out_dir / "manifest.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    return result


def main() -> None:
    p = argparse.ArgumentParser(description="Build Q1 FOLD and UNFOLD SuperMimic references")
    p.add_argument("--crawl", type=Path, default=DEFAULT_CRAWL)
    p.add_argument("--deep-fold", type=Path, default=DEFAULT_DEEP)
    p.add_argument("--lying", type=Path, default=DEFAULT_LYING)
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    a = p.parse_args()
    print(json.dumps(build(a.crawl, a.deep_fold, a.lying, a.model, a.out), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
