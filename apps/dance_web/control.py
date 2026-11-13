"""网页输入仲裁：短时保存速度，技能请求用一次性事件交给 Teleop 状态机。

真机安全配置要求手机网页保持控制会话心跳；网页断开或心跳超时时，服务器会生成一次
``DAMP`` 事件。它与网页上的显式阻尼按钮都是软件保护，不是独立硬件急停。
"""

from __future__ import annotations

import math
import threading
import time

from apps.teleop.gamepad import Pad

ACTION_EDGES = {
    "getup": "GETUP",
    "dance": "DANCE",
    "liedown": "LIEDOWN",
    "fold": "FOLD",
    "damp": "DAMP",
}

SAFETY_PROFILES = ("off", "getup", "walk", "full")
PROFILE_ACTIONS = {
    "off": frozenset(ACTION_EDGES) | {"stand"},
    "getup": frozenset(("getup", "stand", "damp")),
    "walk": frozenset(("getup", "stand", "damp")),
    "full": frozenset(ACTION_EDGES) | {"stand"},
}
PROFILE_AXES = {
    "off": frozenset(("lx", "ly", "rx")),
    "getup": frozenset(),
    "walk": frozenset(("ly",)),
    "full": frozenset(("lx", "ly", "rx")),
}


class WebControl:
    """HTTP 线程写入、50 Hz 控制线程读取。"""

    def __init__(self, drive_timeout: float = 0.5, safety_profile: str = "off", heartbeat_timeout: float = 0.3):
        if drive_timeout <= 0:
            raise ValueError("drive_timeout 必须大于 0")
        if safety_profile not in SAFETY_PROFILES:
            raise ValueError(f"未知安全配置 {safety_profile}")
        if heartbeat_timeout <= 0:
            raise ValueError("heartbeat_timeout 必须大于 0")
        self.drive_timeout = float(drive_timeout)
        self.safety_profile = safety_profile
        self.heartbeat_timeout = float(heartbeat_timeout)
        self.lock = threading.Lock()
        self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
        self.last_drive = 0.0
        self.control_online = False
        self.last_heartbeat = 0.0
        self.failsafe_fired = False
        self.events: list[str] = []
        self.status: dict = {"mode": "STARTING", "message": "控制器启动中"}

    @property
    def safety_enabled(self) -> bool:
        return self.safety_profile != "off"

    @staticmethod
    def _axis(value) -> float:
        value = float(value)
        if not math.isfinite(value) or not -1.0 <= value <= 1.0:
            raise ValueError("摇杆值必须是 [-1, 1] 内的有限数")
        return value

    def drive(self, lx=0.0, ly=0.0, rx=0.0) -> None:
        values = {"lx": self._axis(lx), "ly": self._axis(ly), "rx": self._axis(rx)}
        with self.lock:
            disallowed = set(values) - PROFILE_AXES[self.safety_profile]
            if any(values[name] != 0.0 for name in disallowed):
                raise ValueError(f"安全配置 {self.safety_profile} 禁止该移动方向")
            if self.safety_enabled and any(values.values()) and not self.control_online:
                raise ValueError("网页控制心跳未在线")
            self.axes = values
            self.last_drive = time.monotonic()

    def action(self, name: str) -> None:
        name = str(name).lower()
        with self.lock:
            if name not in PROFILE_ACTIONS[self.safety_profile]:
                raise ValueError(f"安全配置 {self.safety_profile} 禁止动作 {name}")
            if self.safety_enabled and name not in ("stand", "damp") and not self.control_online:
                raise ValueError("网页控制心跳未在线")
            if name == "stand":
                self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
                self.last_drive = time.monotonic()
                return
            try:
                edge = ACTION_EDGES[name]
            except KeyError as e:
                raise ValueError(f"未知动作 {name}") from e
            self.events.append(edge)
            if name == "damp":
                self._disconnect_locked(queue_damp=False)

    def session(self, active: bool) -> None:
        """更新网页控制会话；主动断开或心跳超时都请求阻尼。"""
        if not isinstance(active, bool):
            raise ValueError("active 必须是布尔值")
        if not self.safety_enabled:
            raise ValueError("当前未启用网页断联保护")
        with self.lock:
            if active:
                self.control_online = True
                self.last_heartbeat = time.monotonic()
                self.failsafe_fired = False
            else:
                self._disconnect_locked(queue_damp=self.control_online)

    def _disconnect_locked(self, queue_damp: bool) -> None:
        self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
        self.last_drive = time.monotonic()
        self.control_online = False
        if queue_damp and not self.failsafe_fired:
            self.events.append("DAMP")
            self.failsafe_fired = True

    def _check_heartbeat_locked(self, now: float) -> None:
        if self.safety_enabled and self.control_online and now - self.last_heartbeat > self.heartbeat_timeout:
            self._disconnect_locked(queue_damp=True)

    def sample(self, now: float | None = None) -> tuple[Pad, set[str]]:
        now = time.monotonic() if now is None else float(now)
        with self.lock:
            self._check_heartbeat_locked(now)
            axes = dict(self.axes) if now - self.last_drive <= self.drive_timeout else {"lx": 0.0, "ly": 0.0, "rx": 0.0}
            edges, self.events = set(self.events), []
        return Pad(lx=axes["lx"], ly=axes["ly"], rx=axes["rx"], connected=True), edges

    def set_status(self, status: dict) -> None:
        with self.lock:
            self.status = dict(status)

    def get_status(self) -> dict:
        with self.lock:
            status = dict(self.status)
            status.update(
                safety_profile=self.safety_profile,
                control_online=self.control_online,
                heartbeat_timeout=self.heartbeat_timeout,
            )
            return status
