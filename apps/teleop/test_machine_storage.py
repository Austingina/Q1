from __future__ import annotations

import unittest

import numpy as np

from apps.teleop.gamepad import Pad
from apps.teleop.machine import FOLD, FOLDED, GETUP, LIEDOWN, UNFOLD, Teleop


class _Voice:
    def say(self, text):
        return None


class _Ref:
    def __init__(self, frames=2):
        self.T = frames
        self.started = []

    def start(self, quat, frame=0):
        self.started.append(frame)


class _Robot:
    def torso_quat(self):
        return np.array([1.0, 0.0, 0.0, 0.0])


def _machine(mode: str) -> Teleop:
    """Build only the state needed by the storage transition tests; no ONNX or physics."""
    t = Teleop.__new__(Teleop)
    t.mode, t.msg, t.t_mode = mode, "", 0.0
    t.dt, t.i, t.blend_n = 0.02, 1, 2
    t.sim, t.pad_ok, t.pad_lost = True, True, "damp"
    t.has_skill = t.has_fold = t.has_unfold = True
    t.voice, t.robot, t.recorder = _Voice(), _Robot(), None
    t.target = t.kp = t.fade = None
    t.cmd, t.dh = np.zeros(6), 0.0
    t.fold_after_liedown = False
    t.unfold_ref, t.getup, t.liedown, t.fold_ref = _Ref(), _Ref(), _Ref(), _Ref()
    t.unfold_skill, t.skill, t.fold_skill = object(), object(), object()
    t.pd_unfold = t.pd_skill = t.pd_fold = (np.ones(1), np.ones(1))
    t._act = lambda policy, pd: None
    t._ref_policy = lambda model, ref, frame: (lambda obs: None)
    t._low_level_ready = lambda action: True
    return t


class StorageStateMachineTest(unittest.TestCase):
    def test_unfold_chains_into_existing_getup(self):
        t = _machine(UNFOLD)
        t.tick(Pad(connected=True), set())
        self.assertEqual(t.mode, GETUP)
        self.assertEqual(t.getup.started, [0])
        self.assertIsNotNone(t.fade)

    def test_liedown_chains_into_fold_only_when_requested(self):
        t = _machine(LIEDOWN)
        t.fold_after_liedown = True
        t.tick(Pad(connected=True), set())
        self.assertEqual(t.mode, FOLD)
        self.assertEqual(t.fold_ref.started, [0])

    def test_plain_liedown_does_not_fold(self):
        t = _machine(LIEDOWN)
        damped = []
        t.robot.damp = lambda: damped.append(True)
        t.tick(Pad(connected=True), set())
        self.assertEqual(t.mode, "LYING")
        self.assertEqual(damped, [True])

    def test_fold_finishes_in_held_folded_state(self):
        t = _machine(FOLD)
        t.tick(Pad(connected=True), set())
        self.assertEqual(t.mode, FOLDED)
        self.assertEqual(t.i, t.fold_ref.T - 1)
        self.assertIn("可在 APP 关机", t.msg)


if __name__ == "__main__":
    unittest.main()
