from __future__ import annotations

import json
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

from apps.line_b_web.control import LineBControl
from apps.line_b_web.server import open_server


class LineBControlTest(unittest.TestCase):
    def test_requires_heartbeat_for_motion(self):
        c = LineBControl(mobility_enabled=True)
        with self.assertRaisesRegex(ValueError, "心跳未在线"):
            c.action("getup")
        with self.assertRaisesRegex(ValueError, "心跳未在线"):
            c.drive(ly=0.2)
        c.session(True)
        c.action("getup")
        self.assertEqual(c.sample()[1], {"GETUP"})

    def test_only_forward_and_turn_axes_and_route_b_actions(self):
        c = LineBControl(mobility_enabled=True)
        c.session(True)
        c.set_status({"mode": "WALK"})
        c.drive(ly=0.25, lx=0.0, rx=0.5)
        pad, _ = c.sample()
        self.assertEqual((pad.ly, pad.rx), (0.25, 0.5))
        with self.assertRaisesRegex(ValueError, "禁止横移"):
            c.drive(ly=0.0, lx=0.1)
        with self.assertRaisesRegex(ValueError, "禁止后退与转向组合"):
            c.drive(ly=-1.0, rx=1.0)
        with self.assertRaisesRegex(ValueError, "禁止动作 fold"):
            c.action("fold")
        with self.assertRaisesRegex(ValueError, "禁止动作 unfold"):
            c.action("unfold")

    def test_mobility_is_locked_by_default(self):
        c = LineBControl()
        c.session(True)
        with self.assertRaisesRegex(ValueError, "事故锁定"):
            c.drive(ly=0.1)
        with self.assertRaisesRegex(ValueError, "事故锁定"):
            c.action("dance")
        with self.assertRaisesRegex(ValueError, "事故锁定"):
            c.action("walk")
        self.assertFalse(c.get_status()["mobility_enabled"])

    def test_sim_candidate_can_request_handoff(self):
        c = LineBControl(mobility_enabled=True)
        c.session(True)
        c.action("walk")
        self.assertEqual(c.sample()[1], {"WALK"})

    def test_pose_check_is_read_only_action_gated_to_prone_modes(self):
        c = LineBControl()
        c.session(True)
        c.set_status({"mode": "LYING"})
        c.action("pose_check")
        self.assertEqual(c.sample()[1], {"POSE_CHECK"})
        c.set_status({"mode": "WALK"})
        with self.assertRaisesRegex(ValueError, "LYING / DAMPED"):
            c.action("pose_check")

    def test_nonzero_drive_requires_walk_mode(self):
        c = LineBControl(mobility_enabled=True)
        c.session(True)
        c.set_status({"mode": "STAND_HOLD"})
        with self.assertRaisesRegex(ValueError, "只允许在 WALK"):
            c.drive(ly=1.0)
        c.drive(ly=0.0)
        c.set_status({"mode": "WALK"})
        c.drive(ly=-1.0)
        self.assertEqual(c.sample()[0].ly, -1.0)

    def test_drive_command_expires_to_zero(self):
        c = LineBControl(drive_timeout=0.25, heartbeat_timeout=1.0, mobility_enabled=True)
        with patch("apps.line_b_web.control.time.monotonic", return_value=10.0):
            c.session(True)
            c.set_status({"mode": "WALK"})
            c.drive(ly=1.0, rx=1.0)
        pad, _ = c.sample(10.24)
        self.assertEqual((pad.ly, pad.rx), (1.0, 1.0))
        pad, _ = c.sample(10.26)
        self.assertEqual((pad.ly, pad.rx), (0.0, 0.0))

    def test_drive_direction_audit_counts_presses_and_stop_reason(self):
        c = LineBControl(drive_timeout=0.25, heartbeat_timeout=10.0, mobility_enabled=True)
        with patch("apps.line_b_web.control.time.monotonic", return_value=9.0):
            c.session(True)
        c.set_status({"mode": "WALK"})
        with patch("apps.line_b_web.control.time.monotonic", return_value=10.0):
            c.drive(ly=1.0)
        with patch("apps.line_b_web.control.time.monotonic", return_value=10.1):
            c.drive(ly=1.0)
        with patch("apps.line_b_web.control.time.monotonic", return_value=10.5):
            c.drive(ly=0.0)
        status = c.get_status()
        self.assertEqual(status["drive_input_direction"], "stop")
        self.assertEqual(status["last_nonzero_direction"], "forward")
        self.assertEqual(status["forward_press_count"], 1)
        self.assertEqual(status["backward_press_count"], 0)
        self.assertAlmostEqual(status["last_drive_duration_s"], 0.5)
        self.assertEqual(status["last_drive_stop_reason"], "browser_drive")

        with patch("apps.line_b_web.control.time.monotonic", return_value=11.0):
            c.drive(ly=-1.0)
        self.assertEqual(c.sample(11.3)[0].ly, 0.0)
        status = c.get_status()
        self.assertEqual(status["last_nonzero_direction"], "backward")
        self.assertEqual(status["forward_press_count"], 1)
        self.assertEqual(status["backward_press_count"], 1)
        self.assertAlmostEqual(status["last_drive_duration_s"], 0.3)
        self.assertEqual(status["last_drive_stop_reason"], "drive_timeout")

    def test_turn_and_forward_turn_directions_are_audited_separately(self):
        c = LineBControl(drive_timeout=1.0, heartbeat_timeout=10.0, mobility_enabled=True)
        with patch("apps.line_b_web.control.time.monotonic", return_value=1.0):
            c.session(True)
        c.set_status({"mode": "WALK"})
        commands = ((0.0, 1.0, "turn_left"), (0.0, -1.0, "turn_right"),
                    (1.0, 1.0, "forward_left"), (1.0, -1.0, "forward_right"))
        for i, (ly, rx, expected) in enumerate(commands, start=2):
            with patch("apps.line_b_web.control.time.monotonic", return_value=float(i)):
                c.drive(ly=ly, rx=rx)
            self.assertEqual(c.get_status()["drive_input_direction"], expected)
        c.drive(ly=0.0, rx=0.0)
        status = c.get_status()
        self.assertEqual(status["turn_left_press_count"], 1)
        self.assertEqual(status["turn_right_press_count"], 1)
        self.assertEqual(status["forward_left_press_count"], 1)
        self.assertEqual(status["forward_right_press_count"], 1)
        self.assertEqual((status["drive_input_axis"], status["drive_input_turn_axis"]), (0.0, 0.0))

    def test_dance_has_independent_capability(self):
        c = LineBControl(mobility_enabled=True)
        c.session(True)
        with self.assertRaisesRegex(ValueError, "舞蹈尚未放行"):
            c.action("dance")
        c = LineBControl(mobility_enabled=True, dance_enabled=True)
        c.session(True)
        c.action("dance")
        self.assertEqual(c.sample()[1], {"DANCE"})

    def test_disconnect_and_timeout_zero_drive_without_damp(self):
        c = LineBControl(heartbeat_timeout=0.3, mobility_enabled=True)
        with patch("apps.line_b_web.control.time.monotonic", return_value=10.0):
            c.session(True)
            c.set_status({"mode": "WALK"})
            c.drive(ly=0.5)
        c.session(False)
        p, edges = c.sample(10.1)
        self.assertEqual(p.ly, 0.0)
        self.assertEqual(edges, set())
        self.assertEqual(c.get_status()["last_disconnect_reason"], "page_inactive")
        self.assertFalse(c.sample(10.2)[1])
        with patch("apps.line_b_web.control.time.monotonic", return_value=20.0):
            c.session(True)
        self.assertEqual(c.sample(20.31)[1], set())
        self.assertFalse(c.get_status()["control_online"])
        self.assertEqual(c.get_status()["last_disconnect_reason"], "heartbeat_timeout")
        self.assertEqual(c.get_status()["disconnect_policy"], "zero_and_hold")

    def test_explicit_damp_disconnects(self):
        c = LineBControl()
        c.session(True)
        c.action("damp")
        self.assertFalse(c.get_status()["control_online"])
        self.assertEqual(c.sample()[1], {"DAMP"})
        self.assertEqual(c.get_status()["last_disconnect_reason"], "explicit_damp")

    def test_http_is_separate_and_token_protected(self):
        c = LineBControl()
        server = open_server(c, "127.0.0.1", 0, "route-b-secret")
        root = f"http://127.0.0.1:{server.server_port}"
        try:
            with self.assertRaises(urllib.error.HTTPError) as denied:
                urllib.request.urlopen(root + "/api/status", timeout=2)
            self.assertEqual(denied.exception.code, 403)
            denied.exception.close()
            req = urllib.request.Request(
                root + "/api/session",
                data=json.dumps({"active": True}).encode(),
                headers={"Content-Type": "application/json", "X-Control-Token": "route-b-secret"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=2) as response:
                self.assertEqual(response.status, 200)
            self.assertEqual(c.get_status()["route"], "B")
        finally:
            server.shutdown()
            server.server_close()

    def test_http_candidate_walk_action_is_explicitly_gated(self):
        def post_walk(control):
            server = open_server(control, "127.0.0.1", 0, "route-b-secret")
            try:
                control.session(True)
                req = urllib.request.Request(
                    f"http://127.0.0.1:{server.server_port}/api/action",
                    data=json.dumps({"action": "walk"}).encode(),
                    headers={"Content-Type": "application/json", "X-Control-Token": "route-b-secret"},
                    method="POST",
                )
                try:
                    with urllib.request.urlopen(req, timeout=2) as response:
                        return response.status, control.sample()[1]
                except urllib.error.HTTPError as e:
                    code = e.code
                    e.close()
                    return code, control.sample()[1]
            finally:
                server.shutdown()
                server.server_close()

        self.assertEqual(post_walk(LineBControl()), (400, set()))
        self.assertEqual(post_walk(LineBControl(mobility_enabled=True)), (200, {"WALK"}))


if __name__ == "__main__":
    unittest.main()
