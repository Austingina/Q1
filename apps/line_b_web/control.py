"""线路 B 网页输入仲裁。

动作集合固定，不提供 FOLD / UNFOLD；移动只接受前后轴和转向轴。非零移动只在 WALK 状态接受，
后退与转向不能组合。网页提供的组合只有原地左/右转和前进左/右转。
网页失联或心跳超时只会把移动命令归零并保持当前控制状态；只有显式急停才产生 DAMP。
人工行走/转向与舞蹈使用两个独立能力开关。
"""

from __future__ import annotations

import math
import threading
import time

from apps.teleop.gamepad import Pad

ACTION_EDGES = {
    "pose_check": "POSE_CHECK",
    "getup": "GETUP",
    "walk": "WALK",
    "dance": "DANCE",
    "liedown": "LIEDOWN",
    "damp": "DAMP",
}


class LineBControl:
    def __init__(self, drive_timeout: float = 0.5, heartbeat_timeout: float = 1.0,
                 mobility_enabled: bool = False, dance_enabled: bool = False):
        if drive_timeout <= 0 or heartbeat_timeout <= 0:
            raise ValueError("超时时间必须大于 0")
        self.drive_timeout = float(drive_timeout)
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.mobility_enabled = bool(mobility_enabled)
        self.dance_enabled = bool(dance_enabled)
        self.lock = threading.Lock()
        self.ly = 0.0
        self.rx = 0.0
        self.last_drive = 0.0
        self.control_online = False
        self.last_heartbeat = 0.0
        self.last_disconnect_reason = "not_connected"
        self.drive_direction = "stop"
        self.last_nonzero_direction = "none"
        self.forward_press_count = 0
        self.backward_press_count = 0
        self.turn_left_press_count = 0
        self.turn_right_press_count = 0
        self.forward_left_press_count = 0
        self.forward_right_press_count = 0
        self.drive_press_started = 0.0
        self.last_drive_duration = 0.0
        self.last_drive_stop_reason = "startup"
        self.events: list[str] = []
        self.status: dict = {"route": "B", "mode": "STARTING", "message": "线路 B 控制器启动中"}

    @staticmethod
    def _axis(value) -> float:
        value = float(value)
        if not math.isfinite(value) or not -1.0 <= value <= 1.0:
            raise ValueError("前后轴必须是 [-1, 1] 内的有限数")
        return value

    def drive(self, ly=0.0, rx=0.0, **extra) -> None:
        if any(float(v) != 0.0 for v in extra.values()):
            raise ValueError("线路 B 只允许前后和转向，禁止横移及其他轴")
        ly, rx = self._axis(ly), self._axis(rx)
        if ly < 0.0 and rx != 0.0:
            raise ValueError("线路 B 禁止后退与转向组合")
        with self.lock:
            moving = ly != 0.0 or rx != 0.0
            if moving and not self.mobility_enabled:
                raise ValueError("事故锁定：人工行走和转向尚未放行")
            if moving and not self.control_online:
                raise ValueError("网页控制心跳未在线")
            if moving and self.status.get("mode") != "WALK":
                raise ValueError("非零移动只允许在 WALK 状态下发送")
            self._set_drive_locked(ly, rx, time.monotonic(), "browser_drive")

    @staticmethod
    def _direction(ly: float, rx: float) -> str:
        if ly > 0.0 and rx > 0.0:
            return "forward_left"
        if ly > 0.0 and rx < 0.0:
            return "forward_right"
        if ly > 0.0:
            return "forward"
        if ly < 0.0:
            return "backward"
        if rx > 0.0:
            return "turn_left"
        if rx < 0.0:
            return "turn_right"
        return "stop"

    def _set_drive_locked(self, ly: float, rx: float, now: float, reason: str) -> None:
        direction = self._direction(ly, rx)
        previous = self.drive_direction
        if direction != previous:
            if previous != "stop" and self.drive_press_started:
                self.last_drive_duration = max(0.0, now - self.drive_press_started)
            if direction == "forward":
                self.forward_press_count += 1
            elif direction == "backward":
                self.backward_press_count += 1
            elif direction == "turn_left":
                self.turn_left_press_count += 1
            elif direction == "turn_right":
                self.turn_right_press_count += 1
            elif direction == "forward_left":
                self.forward_left_press_count += 1
            elif direction == "forward_right":
                self.forward_right_press_count += 1
            if direction != "stop":
                self.last_nonzero_direction = direction
                self.drive_press_started = now
            else:
                self.last_drive_stop_reason = reason
            print(
                f"[line-b-input] {previous}->{direction} ly={ly:+.2f} rx={rx:+.2f} "
                f"reason={reason} mode={self.status.get('mode', '?')} "
                f"forward={self.forward_press_count} backward={self.backward_press_count}",
                flush=True,
            )
        self.drive_direction = direction
        self.ly, self.rx = ly, rx
        self.last_drive = now

    def action(self, name: str) -> None:
        name = str(name).lower()
        with self.lock:
            if name == "stand":
                self._set_drive_locked(0.0, 0.0, time.monotonic(), "action:stand")
                return
            if name == "walk" and not self.mobility_enabled:
                raise ValueError("事故锁定：起身到行走交接尚未放行")
            if name == "dance" and not self.dance_enabled:
                raise ValueError("事故锁定：舞蹈尚未放行")
            if name not in ACTION_EDGES:
                raise ValueError(f"线路 B 禁止动作 {name}")
            if name != "damp" and not self.control_online:
                raise ValueError("网页控制心跳未在线")
            if name == "pose_check" and self.status.get("mode") not in ("LYING", "DAMPED"):
                raise ValueError("摆姿检查只允许在 LYING / DAMPED 状态执行")
            self._set_drive_locked(0.0, 0.0, time.monotonic(), f"action:{name}")
            self.events.append(ACTION_EDGES[name])
            if name == "damp":
                self._disconnect_locked("explicit_damp")

    def session(self, active: bool) -> None:
        if not isinstance(active, bool):
            raise ValueError("active 必须是布尔值")
        with self.lock:
            if active:
                self.control_online = True
                self.last_heartbeat = time.monotonic()
            else:
                self._disconnect_locked("page_inactive")

    def _disconnect_locked(self, reason: str) -> None:
        self._set_drive_locked(0.0, 0.0, time.monotonic(), reason)
        self.control_online = False
        self.last_disconnect_reason = reason

    def sample(self, now: float | None = None) -> tuple[Pad, set[str]]:
        now = time.monotonic() if now is None else float(now)
        with self.lock:
            if self.control_online and now - self.last_heartbeat > self.heartbeat_timeout:
                self._disconnect_locked("heartbeat_timeout")
            if (self.ly or self.rx) and now - self.last_drive > self.drive_timeout:
                self._set_drive_locked(0.0, 0.0, now, "drive_timeout")
            ly, rx = self.ly, self.rx
            edges, self.events = set(self.events), []
        return Pad(ly=ly, rx=rx, connected=True), edges

    def set_status(self, status: dict) -> None:
        with self.lock:
            self.status = dict(status)

    def get_status(self) -> dict:
        with self.lock:
            now = time.monotonic()
            active_duration = (max(0.0, now - self.drive_press_started)
                               if self.drive_direction != "stop" else 0.0)
            out = dict(self.status)
            out.update(route="B", safety_profile="line_b", control_online=self.control_online,
                       drive_timeout=self.drive_timeout, heartbeat_timeout=self.heartbeat_timeout,
                       mobility_enabled=self.mobility_enabled, manual_drive_enabled=self.mobility_enabled,
                       dance_enabled=self.dance_enabled,
                       disconnect_policy="zero_and_hold",
                       last_disconnect_reason=self.last_disconnect_reason,
                       drive_input_axis=self.ly,
                       drive_input_turn_axis=self.rx,
                       drive_input_direction=self.drive_direction,
                       last_nonzero_direction=self.last_nonzero_direction,
                       forward_press_count=self.forward_press_count,
                       backward_press_count=self.backward_press_count,
                       turn_left_press_count=self.turn_left_press_count,
                       turn_right_press_count=self.turn_right_press_count,
                       forward_left_press_count=self.forward_left_press_count,
                       forward_right_press_count=self.forward_right_press_count,
                       active_drive_duration_s=active_duration,
                       last_drive_duration_s=self.last_drive_duration,
                       last_drive_stop_reason=self.last_drive_stop_reason)
            return out
