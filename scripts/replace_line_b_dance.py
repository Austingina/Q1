"""Safely replace the complete Line B dance package.

The dance policy is trained together with its motion reference.  This tool
therefore installs a complete package instead of allowing motion.npz alone to
be swapped underneath an unrelated policy.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import uuid
import zipfile
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import yaml


PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_TARGET = PROJECT / "weights" / "q1_smooth_dance"
DEFAULT_BASE = PROJECT / "weights" / "dual_v3b"
BACKUP_ROOT = PROJECT / "weights" / ".dance_backups"
REQUIRED_FILES = ("deploy.yaml", "policy.onnx", "motion.npz", "deploy_motion.npz", "meta.json")
MOTION_KEYS = (
    "fps",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
    "joint_names",
    "body_names",
)
MAX_DANCE_SECONDS = 60.0
MAX_END_JOINT_ERROR = 0.20
MAX_END_JOINT_SPEED = 0.20
MAX_END_ROOT_SPEED = 0.20
MAX_END_ROOT_ANG_SPEED = 0.30
MIN_END_UPRIGHT = 0.90


class InstallError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _array(spec: dict, key: str, label: str) -> np.ndarray:
    if key not in spec:
        raise InstallError(f"{label} 缺少 {key}")
    out = np.asarray(spec[key], dtype=np.float64)
    if not np.isfinite(out).all():
        raise InstallError(f"{label} {key} 包含 NaN/Inf")
    return out


def _same_array(candidate: dict, base: dict, key: str, label: str, atol: float = 1e-6) -> None:
    a, b = _array(candidate, key, label), _array(base, key, "基础包")
    if a.shape != b.shape or not np.allclose(a, b, atol=atol, rtol=0.0):
        raise InstallError(f"{label} {key} 与基础行走包不兼容")


def _obs_layout(spec: dict) -> tuple[int, tuple[tuple[str, int], ...]]:
    try:
        obs = spec["obs"]
        return int(obs["history"]), tuple((str(x["name"]), int(x["dim"])) for x in obs["terms"])
    except (KeyError, TypeError, ValueError) as e:
        raise InstallError("deploy.yaml 的 obs 布局不完整") from e


def _validate_policy_spec(spec: dict, base: dict, package: Path) -> tuple[int, int]:
    if spec.get("robot") != "q1":
        raise InstallError("deploy.yaml robot 必须为 q1")
    if float(spec.get("control_hz", 0.0)) != 50.0:
        raise InstallError("舞蹈包必须使用 50 Hz")
    if "graphs" in spec or "policy" not in spec or "goal" not in spec:
        raise InstallError("输入必须是训练完成的单舞蹈策略包，不能是基础行走包")
    if list(spec.get("joint_names", [])) != list(base.get("joint_names", [])):
        raise InstallError("关节名称或顺序与 dual_v3b 不一致")
    for key in ("default_joint_pos", "action_scale"):
        _same_array(spec, base, key, "舞蹈包")
    for key in ("kp", "kd", "effort_limit"):
        _same_array(spec.get("pd", {}), base.get("pd", {}), key, "舞蹈包 PD")
    if _obs_layout(spec) != _obs_layout(base):
        raise InstallError("观测历史布局与 dual_v3b 不一致")

    try:
        policy = spec["policy"]
        inputs, outputs = policy["inputs"], policy["outputs"]
        obs_dim, goal_dim = int(inputs["obs"][-1]), int(inputs["goal"][-1])
        action_dim = int(outputs["action"][-1])
        target_dim = int(outputs["joint_target"][-1])
        future = int(spec["goal"]["future_frames"])
        anchor = spec["goal"].get("anchor_ori")
        anchor_dim = int(anchor.get("dim", 0)) if isinstance(anchor, dict) else 0
    except (KeyError, TypeError, ValueError) as e:
        raise InstallError("deploy.yaml 的 policy/goal 布局不完整") from e
    expected_obs = _obs_layout(spec)[0] * sum(dim for _, dim in _obs_layout(spec)[1])
    expected_goal = future * 54 + anchor_dim
    joints = len(spec["joint_names"])
    if obs_dim != expected_obs or goal_dim != expected_goal:
        raise InstallError(
            f"ONNX 输入声明不一致：obs={obs_dim}/{expected_obs}, goal={goal_dim}/{expected_goal}"
        )
    if (action_dim, target_dim) != (joints, joints):
        raise InstallError(f"ONNX 输出维度必须为 {joints}/{joints}")
    policy_file = package / str(policy.get("file", ""))
    if policy_file.name != "policy.onnx" or not policy_file.is_file():
        raise InstallError("策略文件必须是存在的 policy.onnx")
    return obs_dim, goal_dim


def _finite(name: str, value: np.ndarray) -> None:
    if np.issubdtype(value.dtype, np.number) and not np.isfinite(value).all():
        raise InstallError(f"motion.npz:{name} 包含 NaN/Inf")


def _validate_motion(package: Path, spec: dict) -> dict:
    motion = package / "motion.npz"
    try:
        data = np.load(motion, allow_pickle=False)
    except Exception as e:
        raise InstallError(f"motion.npz 无法安全读取：{e}") from e
    with data:
        missing = [key for key in MOTION_KEYS if key not in data.files]
        if missing:
            raise InstallError(f"motion.npz 缺少键：{', '.join(missing)}")
        for key in MOTION_KEYS:
            _finite(key, np.asarray(data[key]))

        joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)
        joint_vel = np.asarray(data["joint_vel"], dtype=np.float64)
        body_pos = np.asarray(data["body_pos_w"], dtype=np.float64)
        body_quat = np.asarray(data["body_quat_w"], dtype=np.float64)
        body_lin = np.asarray(data["body_lin_vel_w"], dtype=np.float64)
        body_ang = np.asarray(data["body_ang_vel_w"], dtype=np.float64)
        joint_names = [str(x) for x in data["joint_names"]]
        body_names = [str(x) for x in data["body_names"]]
        fps_values = np.asarray(data["fps"], dtype=np.float64).reshape(-1)

        if len(fps_values) != 1 or not np.isclose(fps_values[0], 50.0, atol=1e-6):
            raise InstallError("motion.npz fps 必须为 50")
        if joint_pos.ndim != 2 or joint_pos.shape[1] != len(spec["joint_names"]):
            raise InstallError(f"joint_pos 必须为 (T, {len(spec['joint_names'])})")
        frames = joint_pos.shape[0]
        if frames < 2:
            raise InstallError("舞蹈至少需要 2 帧")
        duration = (frames - 1) / 50.0
        if duration > MAX_DANCE_SECONDS:
            raise InstallError(f"舞蹈 {duration:.1f} s 超过线路 B 上限 {MAX_DANCE_SECONDS:.0f} s")
        motion_spec = spec.get("motion", {})
        if int(motion_spec.get("frames", -1)) != frames or float(motion_spec.get("fps", 0.0)) != 50.0:
            raise InstallError("deploy.yaml 声明的 motion.frames/fps 与 motion.npz 不一致")
        if joint_names != list(spec["joint_names"]):
            raise InstallError("motion.npz 关节名称或顺序与 deploy.yaml 不一致")
        if len(set(body_names)) != len(body_names) or "pelvis" not in body_names or "torso_link" not in body_names:
            raise InstallError("body_names 必须唯一且包含 pelvis/torso_link")
        bodies = len(body_names)
        expected = {
            "joint_vel": (frames, len(joint_names)),
            "body_pos_w": (frames, bodies, 3),
            "body_quat_w": (frames, bodies, 4),
            "body_lin_vel_w": (frames, bodies, 3),
            "body_ang_vel_w": (frames, bodies, 3),
        }
        actual = {
            "joint_vel": joint_vel.shape,
            "body_pos_w": body_pos.shape,
            "body_quat_w": body_quat.shape,
            "body_lin_vel_w": body_lin.shape,
            "body_ang_vel_w": body_ang.shape,
        }
        for key, shape in expected.items():
            if actual[key] != shape:
                raise InstallError(f"{key} 维度 {actual[key]} 不是 {shape}")
        quat_error = float(np.abs(np.linalg.norm(body_quat, axis=-1) - 1.0).max())
        if quat_error > 1e-3:
            raise InstallError(f"参考四元数未归一化，最大误差 {quat_error:.4g}")

        q0 = np.asarray(spec["default_joint_pos"], dtype=np.float64)
        end_joint_error = float(np.abs(joint_pos[-1] - q0).max())
        end_joint_speed = float(np.abs(joint_vel[-1]).max())
        root = body_names.index("pelvis")
        end_root_speed = float(np.linalg.norm(body_lin[-1, root]))
        end_root_ang_speed = float(np.linalg.norm(body_ang[-1, root]))
        w, x, y, _ = body_quat[-1, root]
        end_upright = float(1.0 - 2.0 * (x * x + y * y))
        if end_joint_error > MAX_END_JOINT_ERROR:
            raise InstallError(
                f"末帧未回到站姿：最大关节差 {end_joint_error:.3f} > {MAX_END_JOINT_ERROR:.2f} rad"
            )
        if end_joint_speed > MAX_END_JOINT_SPEED:
            raise InstallError(f"末帧关节仍在运动：{end_joint_speed:.3f} rad/s")
        if end_root_speed > MAX_END_ROOT_SPEED or end_root_ang_speed > MAX_END_ROOT_ANG_SPEED:
            raise InstallError(
                f"末帧骨盆未静止：线速度 {end_root_speed:.3f} m/s，角速度 {end_root_ang_speed:.3f} rad/s"
            )
        if end_upright < MIN_END_UPRIGHT:
            raise InstallError(f"末帧骨盆不直立：upright={end_upright:.3f}")

        return {
            "frames": frames,
            "duration_s": duration,
            "fps": float(fps_values[0]),
            "end_joint_error": end_joint_error,
            "end_joint_speed": end_joint_speed,
            "end_root_speed": end_root_speed,
            "end_root_ang_speed": end_root_ang_speed,
            "end_upright": end_upright,
        }


def _validate_export_artifacts(package: Path, motion_report: dict, motion_hash: str) -> None:
    try:
        meta = json.loads((package / "meta.json").read_text())
        declared_hash = str(meta["source_motion"]["motion_npz_sha256"])
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as e:
        raise InstallError("meta.json 缺少 source_motion.motion_npz_sha256") from e
    if declared_hash != motion_hash:
        raise InstallError("meta.json 的动作哈希与 motion.npz 不一致，策略/动作可能不是同一训练包")

    try:
        deploy = np.load(package / "deploy_motion.npz", allow_pickle=False)
    except Exception as e:
        raise InstallError(f"deploy_motion.npz 无法安全读取：{e}") from e
    with deploy, np.load(package / "motion.npz", allow_pickle=False) as motion:
        required = {"goal_feat", "anchor_quat_w", "joint_pos", "fps"}
        missing = sorted(required.difference(deploy.files))
        if missing:
            raise InstallError(f"deploy_motion.npz 缺少键：{', '.join(missing)}")
        frames = int(motion_report["frames"])
        expected = {
            "goal_feat": (frames, 54),
            "anchor_quat_w": (frames, 4),
            "joint_pos": (frames, 22),
        }
        for key, shape in expected.items():
            value = np.asarray(deploy[key])
            _finite(f"deploy_motion.npz:{key}", value)
            if value.shape != shape:
                raise InstallError(f"deploy_motion.npz:{key} 维度 {value.shape} 不是 {shape}")
        fps = np.asarray(deploy["fps"], dtype=np.float64).reshape(-1)
        if len(fps) != 1 or not np.isclose(fps[0], 50.0, atol=1e-6):
            raise InstallError("deploy_motion.npz fps 必须为 50")
        if not np.allclose(deploy["joint_pos"], motion["joint_pos"], atol=1e-6, rtol=0.0):
            raise InstallError("deploy_motion.npz 与 motion.npz 不属于同一动作")


def _validate_onnx(package: Path, obs_dim: int, goal_dim: int) -> None:
    try:
        import onnxruntime as ort
    except ImportError as e:
        raise InstallError("缺少 onnxruntime，请通过 ./replace_line_b_dance.command 运行") from e
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    try:
        session = ort.InferenceSession(str(package / "policy.onnx"), options, providers=["CPUExecutionProvider"])
        names = {x.name: x.shape for x in session.get_inputs()}
        outputs = session.run(
            None,
            {
                "obs": np.zeros((1, obs_dim), dtype=np.float32),
                "goal": np.zeros((1, goal_dim), dtype=np.float32),
            },
        )
    except Exception as e:
        raise InstallError(f"policy.onnx 无法推理：{e}") from e
    if set(names) != {"obs", "goal"}:
        raise InstallError(f"policy.onnx 输入必须是 obs/goal，实际为 {sorted(names)}")
    if len(outputs) < 2 or outputs[0].shape != (1, 22) or outputs[1].shape != (1, 22):
        raise InstallError("policy.onnx 实际输出维度不是 (1,22)/(1,22)")
    if not all(np.isfinite(x).all() for x in outputs[:2]):
        raise InstallError("policy.onnx 零输入推理返回 NaN/Inf")


def validate_package(package: Path, base_path: Path = DEFAULT_BASE) -> dict:
    package, base_path = package.resolve(), base_path.resolve()
    if package.is_file() and package.suffix.lower() == ".npz":
        raise InstallError("不能只替换 motion.npz；请选择含新 policy.onnx 的完整训练舞蹈包")
    if not package.is_dir():
        raise InstallError(f"舞蹈包目录不存在：{package}")
    for entry in package.rglob("*"):
        if entry.is_symlink():
            raise InstallError(f"舞蹈包不允许符号链接：{entry.relative_to(package)}")
    missing = [name for name in REQUIRED_FILES if not (package / name).is_file()]
    if missing:
        raise InstallError(f"舞蹈包不完整，缺少：{', '.join(missing)}")
    try:
        spec = yaml.safe_load((package / "deploy.yaml").read_text())
        base = yaml.safe_load((base_path / "deploy.yaml").read_text())
    except Exception as e:
        raise InstallError(f"deploy.yaml 无法读取：{e}") from e
    if not isinstance(spec, dict) or not isinstance(base, dict):
        raise InstallError("deploy.yaml 必须映射为对象")
    obs_dim, goal_dim = _validate_policy_spec(spec, base, package)
    motion = _validate_motion(package, spec)
    motion_hash = sha256(package / "motion.npz")
    _validate_export_artifacts(package, motion, motion_hash)
    _validate_onnx(package, obs_dim, goal_dim)
    return {
        "package": str(package),
        "policy_sha256": sha256(package / "policy.onnx"),
        "motion_sha256": motion_hash,
        "obs_dim": obs_dim,
        "goal_dim": goal_dim,
        **motion,
    }


def _is_zip_symlink(info: zipfile.ZipInfo) -> bool:
    return stat.S_ISLNK((info.external_attr >> 16) & 0xFFFF)


@contextmanager
def package_source(source: Path):
    source = source.expanduser().resolve()
    if source.suffix.lower() != ".zip":
        yield source
        return
    with tempfile.TemporaryDirectory(prefix="q1-dance-package-") as raw:
        root = Path(raw).resolve()
        try:
            with zipfile.ZipFile(source) as archive:
                for info in archive.infolist():
                    target = (root / info.filename).resolve()
                    if root != target and root not in target.parents:
                        raise InstallError(f"ZIP 包含非法路径：{info.filename}")
                    if _is_zip_symlink(info):
                        raise InstallError(f"ZIP 包含符号链接：{info.filename}")
                archive.extractall(root)
        except (OSError, zipfile.BadZipFile) as e:
            raise InstallError(f"ZIP 无法解压：{e}") from e
        candidates = [p.parent for p in root.rglob("deploy.yaml") if all((p.parent / name).is_file() for name in REQUIRED_FILES)]
        if len(candidates) != 1:
            raise InstallError(f"ZIP 内必须恢好包含一个完整舞蹈包，实际找到 {len(candidates)} 个")
        yield candidates[0]


def _copy_package(source: Path, destination: Path) -> None:
    def ignore(_directory: str, names: list[str]) -> set[str]:
        return {name for name in names if name in {".DS_Store", "__pycache__"} or name.endswith(".pyc")}

    shutil.copytree(source, destination, copy_function=shutil.copy2, ignore=ignore)


def _unique_path(parent: Path, name: str) -> Path:
    candidate = parent / name
    i = 1
    while candidate.exists():
        candidate = parent / f"{name}-{i}"
        i += 1
    return candidate


def _write_receipt(package: Path, source: Path, report: dict) -> None:
    receipt = {
        "installed_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": str(source),
        "offline_only": True,
        "robot_synced": False,
        "validation": report,
    }
    (package / "line_b_install.json").write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")


def _run_mujoco_acceptance(package: Path) -> None:
    print("[4/4] 运行完整 MuJoCo 线路 B 验收…", flush=True)
    env = os.environ.copy()
    env["LINE_B_DANCE_PACKAGE"] = str(package)
    try:
        subprocess.run(
            [sys.executable, "-m", "apps.line_b_web.web_handoff_sim_check"],
            cwd=PROJECT,
            env=env,
            check=True,
            timeout=180,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        raise InstallError(f"MuJoCo 验收失败：{e}") from e


def _prune_backups(keep: int) -> None:
    if keep < 1 or not BACKUP_ROOT.exists():
        return
    backups = sorted(
        [p for p in BACKUP_ROOT.iterdir() if p.is_dir() and p.name.startswith("q1_smooth_dance_")],
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    for old in backups[keep:]:
        shutil.rmtree(old)


def install(source: Path, target: Path, keep_backups: int) -> tuple[dict, Path | None]:
    source, target = source.resolve(), target.resolve()
    if source == target:
        raise InstallError("新舞蹈包不能就是当前目标目录")
    target.parent.mkdir(parents=True, exist_ok=True)
    staging = target.parent / f".{target.name}.new-{uuid.uuid4().hex[:10]}"
    backup: Path | None = None
    rejected: Path | None = None
    print("[1/4] 校验新舞蹈包…", flush=True)
    report = validate_package(source)
    print(
        f"      通过：{report['frames']} 帧 / {report['duration_s']:.1f} s，"
        f"policy {report['policy_sha256'][:12]}，motion {report['motion_sha256'][:12]}",
        flush=True,
    )
    try:
        print("[2/4] 复制到同盘临时目录并二次校验…", flush=True)
        _copy_package(source, staging)
        staged_report = validate_package(staging)
        if report["policy_sha256"] != staged_report["policy_sha256"] or report["motion_sha256"] != staged_report["motion_sha256"]:
            raise InstallError("复制后哈希不一致")
        _write_receipt(staging, source, staged_report)

        print("[3/4] 备份旧舞蹈并原子替换…", flush=True)
        if target.exists():
            BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            old_hash = sha256(target / "motion.npz")[:10] if (target / "motion.npz").is_file() else "unknown"
            backup = _unique_path(BACKUP_ROOT, f"q1_smooth_dance_{stamp}_{old_hash}")
            os.replace(target, backup)
        os.replace(staging, target)

        try:
            _run_mujoco_acceptance(target)
        except Exception:
            BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
            rejected = _unique_path(BACKUP_ROOT, f"rejected_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}")
            if target.exists():
                os.replace(target, rejected)
            if backup is not None and backup.exists():
                os.replace(backup, target)
                backup = None
            raise
        _prune_backups(keep_backups)
        return staged_report, backup
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="安全替换线路 B 完整舞蹈包（策略 + 参考动作）",
        epilog="默认只修改 Mac 本地候选，不连接、不同步、不启动真机。",
    )
    parser.add_argument("source", type=Path, help="新舞蹈包目录或 ZIP；不接受单独 motion.npz")
    parser.add_argument("--check-only", action="store_true", help="只校验，不备份、不替换")
    parser.add_argument("--keep-backups", type=int, default=5, help="成功后保留最近备份数（默认 5）")
    parser.add_argument("--target", type=Path, default=DEFAULT_TARGET, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.keep_backups < 1:
        raise SystemExit("--keep-backups 必须至少为 1")
    try:
        with package_source(args.source) as source:
            if args.check_only:
                report = validate_package(source)
                print(json.dumps({"valid": True, "offline_only": True, **report}, ensure_ascii=False, indent=2))
                return
            report, backup = install(source, args.target, args.keep_backups)
    except InstallError as e:
        print(f"\n替换失败：{e}", file=sys.stderr)
        print("当前可用舞蹈未受影响；如已进入替换阶段，脚本已自动回滚。", file=sys.stderr)
        raise SystemExit(1)
    print("\n替换完成：Mac 本地线路 B 候选已通过 MuJoCo 验收。")
    print(f"新舞蹈：{report['duration_s']:.1f} s，motion {report['motion_sha256'][:12]}")
    if backup is not None:
        print(f"旧舞蹈备份：{backup}")
    print("真机未同步，也未启动真机服务；真机使用前仍需单独完成同步和无动作验证。")


if __name__ == "__main__":
    main()
