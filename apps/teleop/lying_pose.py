"""Q1 趴卧姿态分类。

``getup_facedown`` 从双腿位于躯干后方的平趴姿态开始；Q1 关机/收储后常见的
姿态则是髋关节大幅后折、双腿收在躯干两侧。两者的骨盆朝向都表示“趴着”，只看
IMU 无法区分，必须同时检查髋关节。
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

# 实机折叠姿态约为 hip_pitch=-2.91、|hip_yaw|=1.47 rad；平趴起身参考首帧约为
# hip_pitch=-0.12、|hip_yaw|<0.10 rad。阈值留有很大的传感器/摆放余量。
FOLDED_HIP_PITCH_MAX = -2.2
FOLDED_HIP_YAW_MIN = 1.0


def is_folded_prone(joint_names: Sequence[str], joint_pos: Sequence[float]) -> bool:
    """双腿都处于 Q1 收储式折叠姿态时返回 ``True``。

    缺少必要关节或数组长度不一致时保守返回 ``False``，调用方仍会受到后端的
    关节跟踪偏差保护，不能据此绕过安全检查。
    """

    if len(joint_names) != len(joint_pos):
        return False
    index = {name: i for i, name in enumerate(joint_names)}
    required = (
        "left_hip_pitch_joint",
        "right_hip_pitch_joint",
        "left_hip_yaw_joint",
        "right_hip_yaw_joint",
    )
    if any(name not in index for name in required):
        return False
    q = np.asarray(joint_pos, dtype=np.float64)
    pitches = q[[index[required[0]], index[required[1]]]]
    yaws = q[[index[required[2]], index[required[3]]]]
    return bool(np.all(pitches <= FOLDED_HIP_PITCH_MAX) and np.all(np.abs(yaws) >= FOLDED_HIP_YAW_MIN))
