from __future__ import annotations

import argparse
import unittest
from types import SimpleNamespace

import numpy as np

from apps.line_b_web.real_handoff_validation import (
    INFERENCE_BENCHMARK_SAMPLES,
    INFERENCE_BENCHMARK_WARMUP,
    build_parser,
    check_prone_pose,
    evaluate_inference_timing,
    validate_cli_mode,
)


def _args(**overrides):
    values = {
        "backend": "sim",
        "check_only": False,
        "timing_check_only": False,
        "execute_real_handoff": False,
        "max_start_error": 0.7,
        "ramp_s": 3.0,
        "sensor_timeout": 10.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class RealHandoffValidationTest(unittest.TestCase):
    def test_inference_gate_uses_warmed_statistical_sample(self):
        self.assertGreaterEqual(INFERENCE_BENCHMARK_WARMUP, 20)
        self.assertGreaterEqual(INFERENCE_BENCHMARK_SAMPLES, 200)

    def test_real_policy_defaults_to_q1_big_core(self):
        args = build_parser().parse_args([])
        self.assertEqual(args.core, 6)
        self.assertEqual(args.prio, 60)

    def test_real_requires_exactly_one_safe_entry(self):
        with self.assertRaisesRegex(ValueError, "三选一"):
            validate_cli_mode(_args(backend="real"))
        with self.assertRaisesRegex(ValueError, "三选一"):
            validate_cli_mode(_args(backend="real", check_only=True, execute_real_handoff=True))
        validate_cli_mode(_args(backend="real", check_only=True))
        validate_cli_mode(_args(backend="real", timing_check_only=True))
        validate_cli_mode(_args(backend="real", execute_real_handoff=True))

    def test_sim_cannot_consume_real_flags(self):
        with self.assertRaisesRegex(ValueError, "只能与 --backend real"):
            validate_cli_mode(_args(check_only=True))
        with self.assertRaisesRegex(ValueError, "只能与 --backend real"):
            validate_cli_mode(_args(execute_real_handoff=True))
        with self.assertRaisesRegex(ValueError, "只能与 --backend real"):
            validate_cli_mode(_args(timing_check_only=True))

    def test_prone_pose_gate_reports_worst_joint(self):
        skill = SimpleNamespace(joint_names=["a", "b"])
        getup = SimpleNamespace(joint_pos=np.zeros((1, 2)))
        sensed = SimpleNamespace(joint_pos=np.array([0.1, 0.8]))
        result = check_prone_pose(skill, getup, sensed, upright=0.0, max_start_error=0.7)
        self.assertFalse(result.passed)
        self.assertEqual(result.worst_joint, "b")
        self.assertIn("0.800", result.reason)

    def test_inference_timing_gate_accepts_only_50hz_headroom(self):
        passed = evaluate_inference_timing(np.full(50, 10.0), np.full(50, 12.0))
        self.assertTrue(passed.passed)
        slow_mean = evaluate_inference_timing(np.full(50, 16.0), np.full(50, 12.0))
        self.assertFalse(slow_mean.passed)
        self.assertIn("平均推理", slow_mean.reason)
        tail = np.full(50, 10.0)
        tail[-1] = 30.0
        slow_tail = evaluate_inference_timing(tail, np.full(50, 12.0))
        self.assertFalse(slow_tail.passed)
        self.assertIn("p99", slow_tail.reason)


if __name__ == "__main__":
    unittest.main()
