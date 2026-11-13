from __future__ import annotations

import unittest

from apps.teleop.lying_pose import is_folded_prone


NAMES = [
    "left_hip_pitch_joint",
    "left_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_hip_yaw_joint",
]


class LyingPoseTest(unittest.TestCase):
    def test_real_folded_pose_is_detected(self):
        self.assertTrue(is_folded_prone(NAMES, [-2.913, 1.481, -2.910, -1.464]))

    def test_flat_getup_reference_is_not_folded(self):
        self.assertFalse(is_folded_prone(NAMES, [-0.127, -0.047, -0.118, 0.098]))

    def test_one_folded_leg_is_not_enough(self):
        self.assertFalse(is_folded_prone(NAMES, [-2.9, 1.4, -0.2, -0.1]))

    def test_missing_or_misaligned_data_is_safe(self):
        self.assertFalse(is_folded_prone(NAMES[:-1], [-2.9, 1.4, -2.9]))
        self.assertFalse(is_folded_prone(NAMES, [-2.9]))


if __name__ == "__main__":
    unittest.main()
