from __future__ import annotations

import unittest

from apps.line_b_web.offline_handoff import HandoffConfig, smoothstep01


class OfflineHandoffTest(unittest.TestCase):
    def test_selected_default_duration_is_ten_seconds(self):
        self.assertEqual(HandoffConfig().duration_s, 10.0)

    def test_smoothstep_is_bounded_and_monotonic(self):
        values = [smoothstep01(i / 100) for i in range(101)]
        self.assertEqual(values[0], 0.0)
        self.assertEqual(values[-1], 1.0)
        self.assertTrue(all(a <= b for a, b in zip(values, values[1:])))

    def test_config_rejects_non_positive_limits(self):
        with self.assertRaises(ValueError):
            HandoffConfig(duration_s=0).validate()
        with self.assertRaises(ValueError):
            HandoffConfig(target_step_max=-0.1).validate()


if __name__ == "__main__":
    unittest.main()
