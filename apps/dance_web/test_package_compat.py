from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from runtime import Package
from runtime.body_ref import BodyRef

PROJECT = Path(__file__).resolve().parents[2]


class DancePackageCompatibilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = Package.load(PROJECT / "weights/dual_v3b")
        cls.dance = Package.load(PROJECT / "weights/q1_smooth_dance")

    def test_shared_observation_and_joint_contract(self):
        self.assertEqual(self.base.joint_names, self.dance.joint_names)
        np.testing.assert_allclose(self.base.default_joint_pos, self.dance.default_joint_pos)
        np.testing.assert_allclose(self.base.action_scale, self.dance.action_scale)
        np.testing.assert_allclose(self.base.kp, self.dance.kp)
        np.testing.assert_allclose(self.base.kd, self.dance.kd)
        self.assertEqual(self.base.spec["obs"], self.dance.spec["obs"])
        self.assertEqual(self.base.control_dt, self.dance.control_dt)

    def test_dance_reference_matches_policy_goal(self):
        body = BodyRef(self.dance, self.dance.root / "motion.npz")
        goal = body.goal(0, body.anchor_quat[0])
        self.assertEqual(goal.shape, (546,))
        self.assertEqual(body.T, 1016)


if __name__ == "__main__":
    unittest.main()
