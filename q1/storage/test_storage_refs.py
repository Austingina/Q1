from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from apps.teleop.lying_pose import is_folded_prone

PROJECT = Path(__file__).resolve().parents[2]
MOTIONS = PROJECT / "q1/storage/motions"


class StorageReferencesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fold = np.load(MOTIONS / "fold_facedown.npz")
        cls.unfold = np.load(MOTIONS / "unfold_facedown.npz")
        cls.lying = np.load(PROJECT / "apps/ref/skills/liedown_facedown.npz")
        cls.names = [str(x) for x in cls.fold["joint_names"]]

    @classmethod
    def tearDownClass(cls):
        cls.fold.close()
        cls.unfold.close()
        cls.lying.close()

    def test_contract_and_finite_values(self):
        self.assertEqual(self.fold["joint_pos"].shape, (640, 22))
        self.assertEqual(self.fold["body_pos_w"].shape, (640, 26, 3))
        for key in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"):
            self.assertTrue(np.isfinite(self.fold[key]).all(), key)
            self.assertTrue(np.isfinite(self.unfold[key]).all(), key)
        np.testing.assert_allclose(np.linalg.norm(self.fold["body_quat_w"], axis=-1), 1.0, atol=2e-6)

    def test_fold_starts_at_existing_liedown_and_ends_deep_folded(self):
        np.testing.assert_allclose(self.fold["joint_pos"][0], self.lying["joint_pos"][-1], atol=1e-6)
        self.assertFalse(is_folded_prone(self.names, self.fold["joint_pos"][0]))
        self.assertTrue(is_folded_prone(self.names, self.fold["joint_pos"][-1]))

    def test_unfold_is_exact_time_reverse_and_ends_at_getup_handoff(self):
        for key in ("joint_pos", "body_pos_w", "body_quat_w"):
            np.testing.assert_allclose(self.unfold[key], self.fold[key][::-1], atol=1e-7)
        for key in ("joint_vel", "body_lin_vel_w", "body_ang_vel_w"):
            np.testing.assert_allclose(self.unfold[key], -self.fold[key][::-1], atol=1e-6)
        self.assertTrue(is_folded_prone(self.names, self.unfold["joint_pos"][0]))
        np.testing.assert_allclose(self.unfold["joint_pos"][-1], self.lying["joint_pos"][-1], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
