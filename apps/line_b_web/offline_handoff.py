"""线路 B 交接实验的离线便捷入口。

正式状态机已经包含默认关闭的交接实现；本包装器强制为每个实例启用该实现，
而正式状态机本身会拒绝任何非 MuJoCo 后端。
"""

from __future__ import annotations

from apps.line_b_web.machine import (
    DAMPED,
    WALK_HANDOFF,
    WALK_SETTLE,
    HandoffConfig,
    HandoffMetrics,
    LineBMachine,
    smoothstep01,
)


class OfflineHandoffMachine(LineBMachine):
    def __init__(self, *args, handoff: HandoffConfig | None = None, **kwargs):
        if "experimental_handoff" in kwargs:
            raise TypeError("请使用 handoff 参数配置离线交接")
        super().__init__(*args, experimental_handoff=handoff or HandoffConfig(), **kwargs)


__all__ = [
    "DAMPED",
    "WALK_HANDOFF",
    "WALK_SETTLE",
    "HandoffConfig",
    "HandoffMetrics",
    "OfflineHandoffMachine",
    "smoothstep01",
]
