from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import replace_line_b_dance as tool


REPORT = {
    "package": "test",
    "policy_sha256": "a" * 64,
    "motion_sha256": "b" * 64,
    "obs_dim": 720,
    "goal_dim": 546,
    "frames": 1016,
    "duration_s": 20.3,
    "fps": 50.0,
    "end_joint_error": 0.0,
    "end_joint_speed": 0.0,
    "end_root_speed": 0.0,
    "end_root_ang_speed": 0.0,
    "end_upright": 1.0,
}


def package(path: Path, marker: str) -> Path:
    path.mkdir(parents=True)
    (path / "motion.npz").write_bytes(marker.encode())
    (path / "marker.txt").write_text(marker)
    return path


class ReplaceLineBDanceTest(unittest.TestCase):
    def test_atomic_install_keeps_backup(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            source = package(root / "source", "new")
            target = package(root / "weights/q1_smooth_dance", "old")
            backups = root / "backups"
            with patch.object(tool, "BACKUP_ROOT", backups), \
                    patch.object(tool, "validate_package", return_value=REPORT.copy()), \
                    patch.object(tool, "_run_mujoco_acceptance") as acceptance:
                report, backup = tool.install(source, target, keep_backups=5)
            acceptance.assert_called_once_with(target.resolve())
            self.assertEqual(report["motion_sha256"], "b" * 64)
            self.assertEqual((target / "marker.txt").read_text(), "new")
            self.assertIsNotNone(backup)
            self.assertEqual((backup / "marker.txt").read_text(), "old")
            self.assertTrue((target / "line_b_install.json").is_file())

    def test_acceptance_failure_restores_old_package(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            source = package(root / "source", "new")
            target = package(root / "weights/q1_smooth_dance", "old")
            backups = root / "backups"
            with patch.object(tool, "BACKUP_ROOT", backups), \
                    patch.object(tool, "validate_package", return_value=REPORT.copy()), \
                    patch.object(tool, "_run_mujoco_acceptance", side_effect=tool.InstallError("sim failed")):
                with self.assertRaisesRegex(tool.InstallError, "sim failed"):
                    tool.install(source, target, keep_backups=5)
            self.assertEqual((target / "marker.txt").read_text(), "old")
            rejected = list(backups.glob("rejected_*"))
            self.assertEqual(len(rejected), 1)
            self.assertEqual((rejected[0] / "marker.txt").read_text(), "new")


if __name__ == "__main__":
    unittest.main()
