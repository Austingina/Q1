from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np

MODULE = Path(__file__).with_name("native_capture_to_motion.py")
SPEC = importlib.util.spec_from_file_location("native_capture_to_motion_under_test", MODULE)
module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(module)


class NativeCaptureConversionTest(unittest.TestCase):
    def test_nearest_resample(self):
        t = np.arange(0.0, 1.001, 0.005)
        values = np.stack([t, t * 2], axis=1)
        grid, out = module.nearest_resample(t, values, 50.0)
        self.assertEqual(out.shape, (51, 2))
        np.testing.assert_allclose(out[:, 0], grid, atol=0.0026)

    def test_quaternion_sign_is_continuous(self):
        q = np.array([[1.0, 0, 0, 0], [-1.0, 0, 0, 0], [0.999, 0.01, 0, 0]])
        out = module.normalize_quat(q)
        self.assertTrue(np.all(np.sum(out[:-1] * out[1:], axis=1) >= 0))
        np.testing.assert_allclose(np.linalg.norm(out, axis=1), 1.0)

    def test_bad_timestamps_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "时间戳"):
            module.nearest_resample(np.array([0.0, 0.0]), np.zeros((2, 1)), 50.0)


if __name__ == "__main__":
    unittest.main()
