from __future__ import annotations

import unittest
import json
import urllib.error
import urllib.request
from unittest.mock import patch

from apps.dance_web.control import WebControl
from apps.dance_web.server import open_server


class WebControlTest(unittest.TestCase):
    def test_drive_and_timeout(self):
        c = WebControl(0.5)
        with patch("apps.dance_web.control.time.monotonic", return_value=10.0):
            c.drive(lx=0.25, ly=0.5, rx=-0.75)
        p, edges = c.sample(10.4)
        self.assertEqual((p.lx, p.ly, p.rx), (0.25, 0.5, -0.75))
        self.assertFalse(edges)
        p, _ = c.sample(10.6)
        self.assertEqual((p.lx, p.ly, p.rx), (0.0, 0.0, 0.0))

    def test_actions_are_one_shot(self):
        c = WebControl()
        c.action("dance")
        _, edges = c.sample()
        self.assertEqual(edges, {"DANCE"})
        self.assertFalse(c.sample()[1])

    def test_fold_is_a_separate_one_shot_action(self):
        c = WebControl(safety_profile="full")
        c.session(True)
        c.action("fold")
        self.assertEqual(c.sample()[1], {"FOLD"})
        self.assertFalse(c.sample()[1])

    def test_stand_zeros_drive(self):
        c = WebControl()
        c.drive(1, 1, 1)
        c.action("stand")
        p, edges = c.sample()
        self.assertEqual((p.lx, p.ly, p.rx), (0.0, 0.0, 0.0))
        self.assertFalse(edges)

    def test_rejects_bad_axis(self):
        c = WebControl()
        with self.assertRaises(ValueError):
            c.drive(2, 0, 0)
        with self.assertRaises(ValueError):
            c.drive(float("nan"), 0, 0)

    def test_getup_profile_requires_online_session_and_rejects_drive_and_dance(self):
        c = WebControl(safety_profile="getup")
        with self.assertRaisesRegex(ValueError, "心跳未在线"):
            c.action("getup")
        c.session(True)
        c.action("getup")
        self.assertEqual(c.sample()[1], {"GETUP"})
        with self.assertRaisesRegex(ValueError, "禁止"):
            c.drive(0, 0.1, 0)
        with self.assertRaisesRegex(ValueError, "禁止"):
            c.action("dance")

    def test_session_disconnect_and_timeout_each_damp_once(self):
        c = WebControl(safety_profile="walk", heartbeat_timeout=0.3)
        with patch("apps.dance_web.control.time.monotonic", return_value=10.0):
            c.session(True)
            c.drive(0, 0.5, 0)
        c.session(False)
        p, edges = c.sample(10.1)
        self.assertEqual((p.lx, p.ly, p.rx), (0.0, 0.0, 0.0))
        self.assertEqual(edges, {"DAMP"})
        self.assertFalse(c.sample(10.2)[1])

        with patch("apps.dance_web.control.time.monotonic", return_value=20.0):
            c.session(True)
            c.drive(0, 0.5, 0)
        p, edges = c.sample(20.31)
        self.assertEqual((p.lx, p.ly, p.rx), (0.0, 0.0, 0.0))
        self.assertEqual(edges, {"DAMP"})
        self.assertFalse(c.get_status()["control_online"])
        self.assertFalse(c.sample(21.0)[1])

    def test_walk_profile_only_accepts_forward_axis(self):
        c = WebControl(safety_profile="walk")
        c.session(True)
        c.drive(0, 0.25, 0)
        self.assertEqual(c.sample()[0].ly, 0.25)
        with self.assertRaisesRegex(ValueError, "禁止"):
            c.drive(0.1, 0, 0)
        with self.assertRaisesRegex(ValueError, "禁止"):
            c.drive(0, 0, 0.1)

    def test_explicit_damp_disconnects_session(self):
        c = WebControl(safety_profile="full")
        c.session(True)
        c.action("damp")
        self.assertFalse(c.get_status()["control_online"])
        self.assertEqual(c.sample()[1], {"DAMP"})

    def test_http_token_and_action(self):
        c = WebControl()
        server = open_server(c, "127.0.0.1", 0, "secret")
        root = f"http://127.0.0.1:{server.server_port}"
        try:
            with self.assertRaises(urllib.error.HTTPError) as denied:
                urllib.request.urlopen(root + "/api/status", timeout=2)
            self.assertEqual(denied.exception.code, 403)
            denied.exception.close()
            req = urllib.request.Request(
                root + "/api/action",
                data=json.dumps({"action": "dance"}).encode(),
                headers={"Content-Type": "application/json", "X-Control-Token": "secret"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=2) as response:
                self.assertEqual(response.status, 200)
            self.assertEqual(c.sample()[1], {"DANCE"})
        finally:
            server.shutdown()
            server.server_close()

    def test_http_session_endpoint(self):
        c = WebControl(safety_profile="getup")
        server = open_server(c, "127.0.0.1", 0, "secret")
        root = f"http://127.0.0.1:{server.server_port}"
        try:
            req = urllib.request.Request(
                root + "/api/session",
                data=json.dumps({"active": True}).encode(),
                headers={"Content-Type": "application/json", "X-Control-Token": "secret"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=2) as response:
                self.assertEqual(response.status, 200)
            self.assertTrue(c.get_status()["control_online"])
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
