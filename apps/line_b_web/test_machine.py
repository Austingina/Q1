from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from apps.line_b_web.machine import (
    DAMPED,
    DANCE,
    GETUP,
    LIEDOWN,
    LYING,
    STAND_HOLD,
    STRAIGHTEN,
    WALK,
    WALK_HANDOFF,
    WALK_SETTLE,
    HandoffConfig,
    HandoffMetrics,
    LineBMachine,
)
from apps.line_b_web.control import LineBControl
from apps.teleop.gamepad import Pad
from apps.teleop.command import ACC, BACK, FWD, TURN


class _Ref:
    T = 2

    def start(self, quat):
        return None


class _Robot:
    def __init__(self):
        self.damped = 0

    def damp(self):
        self.damped += 1

    def torso_quat(self):
        return np.array([1.0, 0.0, 0.0, 0.0])

    def upright(self):
        return 1.0

    def sense(self):
        return SimpleNamespace(base_ang_vel=np.zeros(3))


def _machine(mode: str) -> LineBMachine:
    m = LineBMachine.__new__(LineBMachine)
    m.mode, m.msg, m.t_mode = mode, "", 0.0
    m.dt, m.i, m.blend_n, m.settle_s = 0.02, 1, 2, 0.0
    m.speed = 0.5
    m.dance_enabled, m.walk_msg = True, "test walk"
    m.robot, m.sim = _Robot(), True
    m.getup = m.liedown = m.dance_ref = _Ref()
    m.skill = m.dance_skill = object()
    m.pd_skill = m.pd_dance = m.pd_base = (np.ones(1), np.ones(1))
    m.fade = None
    m.after_straighten = None
    m.cmd = np.zeros(6)
    m.target = m.kp = None
    m.handoff = None
    m.pose_check_result = {"state": "not_run", "checked_count": 0}
    m.handoff_label = "仅仿真实验"
    m.voice = type("Voice", (), {"say": lambda self, text: None})()
    m._act = lambda policy, pd: None
    m._ref_policy = lambda model, ref, frame: (lambda obs: None)
    return m


class LineBMachineTest(unittest.TestCase):
    def test_getup_finishes_in_skill_hold_not_walk(self):
        m = _machine(GETUP)
        m.tick(Pad(), set())
        self.assertEqual(m.mode, STAND_HOLD)
        self.assertIsNone(m.fade)

    def test_liedown_finishes_in_damped_lying(self):
        m = _machine(LIEDOWN)
        m.tick(Pad(), set())
        self.assertEqual(m.mode, LYING)
        self.assertEqual(m.robot.damped, 1)

    def test_dance_returns_to_walk(self):
        m = _machine(DANCE)
        m.tick(Pad(), set())
        self.assertEqual(m.mode, WALK)
        self.assertIsNotNone(m.fade)

    def test_dance_request_zeroes_and_settles_before_start(self):
        m = _machine(WALK)
        m.cmd[0], m.cmd[2] = 0.30, 0.50
        m._tele_policy = lambda obs: None
        m.tick(Pad(), {"DANCE"})
        self.assertEqual(m.mode, STRAIGHTEN)
        self.assertEqual(m.after_straighten, "dance")
        self.assertAlmostEqual(m.cmd[0], 0.30)
        self.assertAlmostEqual(m.cmd[2], 0.50)
        for _ in range(30):
            m.tick(Pad(), set())
            if m.mode == DANCE:
                break
        self.assertEqual(m.mode, DANCE)
        np.testing.assert_allclose(m.cmd, 0.0, atol=1e-9)

    def test_manual_drive_has_separate_forward_and_backward_caps(self):
        m = _machine(WALK)
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        for _ in range(20):
            m.tick(Pad(ly=1.0), set())
        self.assertAlmostEqual(m.cmd[0], 0.30)
        for _ in range(20):
            m.tick(Pad(), set())
        self.assertAlmostEqual(m.cmd[0], 0.0)
        for _ in range(20):
            m.tick(Pad(ly=-1.0), set())
        self.assertAlmostEqual(m.cmd[0], -0.20)
        np.testing.assert_allclose(m.cmd[1:], 0.0)

    def test_manual_drive_uses_original_single_scale_and_ramp(self):
        m = _machine(WALK)
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        m.tick(Pad(ly=1.0, rx=1.0), set())
        self.assertAlmostEqual(m.speed, 0.5)
        self.assertAlmostEqual(m.cmd[0], ACC[0] * m.dt)
        self.assertAlmostEqual(m.cmd[2], ACC[2] * m.dt)
        self.assertAlmostEqual(m.speed * FWD, 0.30)
        self.assertAlmostEqual(m.speed * BACK, 0.20)
        self.assertAlmostEqual(m.speed * TURN, 0.50)

    def test_manual_turn_and_forward_turn_use_separate_yaw_cap(self):
        m = _machine(WALK)
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        for _ in range(20):
            m.tick(Pad(rx=1.0), set())
        self.assertAlmostEqual(m.cmd[0], 0.0)
        self.assertAlmostEqual(m.cmd[2], 0.50)
        for _ in range(20):
            m.tick(Pad(), set())
        for _ in range(20):
            m.tick(Pad(ly=1.0, rx=-1.0), set())
        self.assertAlmostEqual(m.cmd[0], 0.30)
        self.assertAlmostEqual(m.cmd[2], -0.50)
        np.testing.assert_allclose(m.cmd[[1, 3, 4, 5]], 0.0)

    def test_manual_drive_keeps_dance_locked(self):
        m = _machine(WALK)
        m.dance_enabled = False
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        m.tick(Pad(), {"DANCE"})
        self.assertEqual(m.mode, WALK)
        self.assertIn("舞蹈尚未放行", m.msg)

    def test_web_disconnect_keeps_stand_hold_active(self):
        control = LineBControl(mobility_enabled=True)
        control.session(True)
        control.session(False)
        pad, edges = control.sample()
        m = _machine(STAND_HOLD)
        m.tick(pad, edges)
        self.assertEqual(m.mode, STAND_HOLD)
        self.assertEqual(m.robot.damped, 0)
        self.assertEqual(edges, set())

    def test_web_disconnect_zeroes_walk_without_damp(self):
        control = LineBControl(mobility_enabled=True)
        control.session(True)
        control.set_status({"mode": WALK})
        control.drive(ly=1.0)
        control.session(False)
        m = _machine(WALK)
        m.cmd[0] = 0.08
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        for _ in range(10):
            pad, edges = control.sample()
            m.tick(pad, edges)
        self.assertEqual(m.mode, WALK)
        self.assertAlmostEqual(m.cmd[0], 0.0)
        self.assertEqual(m.robot.damped, 0)

    def test_walk_handoff_is_locked_by_default(self):
        m = _machine(STAND_HOLD)
        m.tick(Pad(), {"WALK"})
        self.assertEqual(m.mode, STAND_HOLD)
        self.assertIn("未启用", m.msg)

    def test_explicit_sim_handoff_accepts_walk_event(self):
        m = _machine(STAND_HOLD)
        m.handoff = HandoffConfig()
        started = []
        m._start_handoff = lambda: started.append(True)
        m.tick(Pad(), {"WALK"})
        self.assertEqual(started, [True])

    def test_handoff_step_runs_only_one_policy_inference(self):
        m = _machine(WALK_HANDOFF)
        m.handoff = HandoffConfig(duration_s=1.0)
        m.handoff_metrics = HandoffMetrics()
        m.handoff_n = 50
        m.handoff_i = 0
        m.handoff_anchor_target = np.zeros(1)
        m.handoff_skill_target = np.zeros(1)
        m.handoff_base_target = np.array([0.1])
        m.handoff_prev_target = np.zeros(1)
        m.s = SimpleNamespace(obs=lambda: np.zeros((1, 1)))
        calls = []
        m._ref_policy = lambda model, ref, frame: (
            lambda obs: (calls.append("skill") or (np.zeros(1), np.zeros(1)))
        )
        m._zero_speed_base_policy = lambda obs: (calls.append("base") or (np.zeros(1), np.array([0.1])))
        m._apply_handoff = lambda target, kp, kd: None
        m._measure_handoff = lambda target: True
        m._handoff_step()
        self.assertEqual(calls, ["skill"])
        m._handoff_step()
        self.assertEqual(calls, ["skill", "base"])

    def test_handoff_anchor_is_previous_applied_skill_target(self):
        m = _machine(STAND_HOLD)
        m.handoff = HandoffConfig()
        m.base = SimpleNamespace(action_scale=np.array([0.5]))
        m.s = SimpleNamespace(q0=np.array([1.0]), last_action=np.array([0.4]), obs=lambda: np.zeros((1, 1)))
        calls = []
        m._zero_speed_base_policy = lambda obs: (calls.append("base") or (np.zeros(1), np.array([1.1])))
        m._start_handoff()
        self.assertEqual(calls, ["base"])
        np.testing.assert_allclose(m.handoff_anchor_target, [1.2])
        np.testing.assert_allclose(m.handoff_skill_target, [1.2])
        np.testing.assert_allclose(m.handoff_base_target, [1.1])
        np.testing.assert_allclose(m.handoff_prev_target, [1.2])

    def test_handoff_modes_honor_damp_immediately(self):
        for mode in (WALK_HANDOFF, WALK_SETTLE):
            with self.subTest(mode=mode):
                m = _machine(mode)
                m.tick(Pad(), {"DAMP"})
                self.assertEqual(m.mode, DAMPED)
                self.assertEqual(m.robot.damped, 1)

    def test_experimental_handoff_rejects_non_mujoco_backend(self):
        pkg = SimpleNamespace(
            joint_names=["joint"],
            default_joint_pos=np.zeros(1),
            action_scale=np.ones(1),
            control_dt=0.02,
        )
        with self.assertRaisesRegex(ValueError, "只允许 MuJoCoBackend"):
            LineBMachine(
                pkg,
                pkg,
                _Robot(),
                Path("unused_getup.npz"),
                Path("unused_liedown.npz"),
                pkg,
                Path("unused_dance.npz"),
                speed_scale=0.1,
                experimental_handoff=HandoffConfig(),
            )

    @patch("apps.line_b_web.machine.BodyRef")
    @patch("apps.line_b_web.machine.SkillModel")
    @patch("apps.line_b_web.machine.TeleGoal")
    @patch("apps.line_b_web.machine.Session")
    def test_dedicated_capability_allows_non_mujoco_handoff(self, session, goal, model, body_ref):
        pkg = SimpleNamespace(
            joint_names=["joint"],
            default_joint_pos=np.zeros(1),
            action_scale=np.ones(1),
            control_dt=0.02,
            kp=np.ones(1),
            kd=np.ones(1),
        )
        m = LineBMachine(
            pkg,
            pkg,
            _Robot(),
            Path("unused_getup.npz"),
            Path("unused_liedown.npz"),
            pkg,
            Path("unused_dance.npz"),
            speed_scale=0.0,
            experimental_handoff=HandoffConfig(),
            allow_real_handoff_validation=True,
        )
        self.assertFalse(m.sim)
        self.assertEqual(m.handoff_label, "一次性真机零速验收")

    @patch("apps.line_b_web.machine.BodyRef")
    @patch("apps.line_b_web.machine.SkillModel")
    @patch("apps.line_b_web.machine.TeleGoal")
    @patch("apps.line_b_web.machine.Session")
    def test_manual_drive_capability_is_separate_from_dance(self, session, goal, model, body_ref):
        pkg = SimpleNamespace(
            joint_names=["joint"],
            default_joint_pos=np.zeros(1),
            action_scale=np.ones(1),
            control_dt=0.02,
            kp=np.ones(1),
            kd=np.ones(1),
        )
        m = LineBMachine(
            pkg,
            pkg,
            _Robot(),
            Path("unused_getup.npz"),
            Path("unused_liedown.npz"),
            pkg,
            Path("unused_dance.npz"),
            speed_scale=0.5,
            experimental_handoff=HandoffConfig(),
            allow_real_manual_drive=True,
        )
        self.assertFalse(m.sim)
        self.assertEqual(m.handoff_label, "线路 B 人工行走和转向")
        self.assertFalse(m.dance_enabled)
        self.assertAlmostEqual(m.speed * FWD, 0.30)
        self.assertAlmostEqual(m.speed * BACK, 0.20)
        self.assertAlmostEqual(m.speed * TURN, 0.50)

    def test_dedicated_capability_requires_handoff_config(self):
        pkg = SimpleNamespace(
            joint_names=["joint"],
            default_joint_pos=np.zeros(1),
            action_scale=np.ones(1),
            control_dt=0.02,
        )
        with self.assertRaisesRegex(ValueError, "capability"):
            LineBMachine(
                pkg,
                pkg,
                _Robot(),
                Path("unused_getup.npz"),
                Path("unused_liedown.npz"),
                pkg,
                Path("unused_dance.npz"),
                speed_scale=0.0,
                allow_real_handoff_validation=True,
            )

    def test_storage_events_are_not_states_or_transitions(self):
        m = _machine(WALK)
        m._tele_policy = lambda obs: None
        m._act = lambda policy, pd: None
        m.tick(Pad(), {"FOLD", "UNFOLD"})
        self.assertEqual(m.mode, WALK)

    @patch("apps.line_b_web.machine.is_folded_prone", return_value=False)
    def test_web_pose_check_passes_without_actuation(self, folded):
        m = _machine(LYING)
        m.base = SimpleNamespace(joint_names=["joint"])
        m.getup = SimpleNamespace(joint_pos=np.zeros((1, 1)))
        calls = []
        m.robot = SimpleNamespace(
            sense=lambda: SimpleNamespace(joint_pos=np.array([0.2])),
            upright=lambda: 0.02,
            reset=lambda *args: calls.append("reset"),
            step=lambda *args: calls.append("step"),
            set_pd=lambda *args: calls.append("set_pd"),
        )
        m.tick(Pad(), {"POSE_CHECK"})
        self.assertEqual(m.mode, LYING)
        self.assertTrue(m.pose_check_result["passed"])
        self.assertAlmostEqual(m.pose_check_result["max_joint_error"], 0.2)
        self.assertEqual(calls, [])
        self.assertIn("未下发动作", m.msg)

    @patch("apps.line_b_web.machine.is_folded_prone", return_value=True)
    def test_web_pose_check_reports_folded_without_actuation(self, folded):
        m = _machine(DAMPED)
        m.base = SimpleNamespace(joint_names=["joint"])
        m.getup = SimpleNamespace(joint_pos=np.zeros((1, 1)))
        m.robot = SimpleNamespace(
            sense=lambda: SimpleNamespace(joint_pos=np.zeros(1)),
            upright=lambda: 0.01,
        )
        m.tick(Pad(), {"POSE_CHECK"})
        self.assertFalse(m.pose_check_result["passed"])
        self.assertTrue(m.pose_check_result["folded"])
        self.assertIn("深度折叠", m.pose_check_result["reason"])

    @patch("apps.line_b_web.machine.vendor_mc_running", return_value=False)
    @patch("apps.line_b_web.machine.is_folded_prone", return_value=True)
    def test_folded_pose_is_rejected_before_reset(self, folded, vendor):
        reset = []
        m = LineBMachine.__new__(LineBMachine)
        m.sim, m.msg = False, ""
        m.base = SimpleNamespace(joint_names=["joint"])
        m.getup = SimpleNamespace(joint_pos=np.zeros((1, 1)))
        m.robot = SimpleNamespace(
            sense=lambda: SimpleNamespace(joint_pos=np.zeros(1)),
            upright=lambda: 0.0,
            reset=lambda q: reset.append(q),
        )
        m._start_getup()
        self.assertIn("深度折叠", m.msg)
        self.assertFalse(reset)

    @patch("apps.line_b_web.machine.vendor_mc_running", return_value=False)
    @patch("apps.line_b_web.machine.is_folded_prone", return_value=False)
    def test_far_flat_pose_is_rejected_before_reset(self, folded, vendor):
        reset = []
        m = LineBMachine.__new__(LineBMachine)
        m.sim, m.msg = False, ""
        m.base = SimpleNamespace(joint_names=["joint"])
        m.getup = SimpleNamespace(joint_pos=np.zeros((1, 1)))
        m.robot = SimpleNamespace(
            sense=lambda: SimpleNamespace(joint_pos=np.array([0.8])),
            upright=lambda: 0.0,
            reset=lambda q: reset.append(q),
        )
        m._start_getup()
        self.assertIn("0.80 > 0.70", m.msg)
        self.assertFalse(reset)


if __name__ == "__main__":
    unittest.main()
