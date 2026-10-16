"""
VR 到机械臂的基础变换工具.

本文件只放不依赖 ROS node 状态的纯函数。VR 到 TCP 的相对映射与
关节限位裁剪都放在这里，供 mapper 和测试复用。
"""

import math
from copy import deepcopy

import numpy as np

from teleop_core.kinematics import _matrix_to_quat, _normalize_quat, _quat_inverse, _quat_multiply


# 四元数转欧拉角
def quaternion_to_euler(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# 检查位姿是否有限
def finite_pose(pose):
    values = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    return all(math.isfinite(value) for value in values)


def shortest_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scaled_pose(pose, reference_pose, position_scale):
    if reference_pose is None:
        return pose

    scaled = deepcopy(pose)
    # 缩放后位置 = 参考位置 + 当前相对参考位置的变化量 × 缩放比例
    scaled.position.x = reference_pose.position.x + (
        pose.position.x - reference_pose.position.x
    ) * position_scale
    scaled.position.y = reference_pose.position.y + (
        pose.position.y - reference_pose.position.y
    ) * position_scale
    scaled.position.z = reference_pose.position.z + (
        pose.position.z - reference_pose.position.z
    ) * position_scale
    return scaled


def vr_axis_maps(vr_rot_flat):
    """
    把 row-major 长度9/16序列转成平移/旋转轴映射矩阵;非法时返回单位阵.
    """
    try:
        values = np.asarray(vr_rot_flat, dtype=float)
    except (TypeError, ValueError):
        identity = np.eye(3, dtype=float)
        return identity, identity

    # 参数必须刚好包含 9 或 16 个有限数，按 row-major 还原。
    if not np.isfinite(values).all():
        identity = np.eye(3, dtype=float)
        return identity, identity
    if values.size == 9:
        rotation = values.reshape(3, 3)
        return rotation, rotation

    if values.size == 16:
        matrix = values.reshape(4, 4)
        rotation = matrix[:3, :3]
        position = matrix[:3, 3:4] * np.abs(rotation)
        return position, rotation
    identity = np.eye(3, dtype=float)
    return identity, identity


def _quat_to_matrix(quat):
    x, y, z, w = _normalize_quat(quat)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def _slerp_from_identity(q_delta, t):
    # 归一化并取 w >= 0 的等价四元数，保持 identity 到 q_delta 的最短路径。
    q = _normalize_quat(q_delta)
    if q[3] < 0.0:
        q = -q

    vector = q[:3]
    sin_half = np.linalg.norm(vector)
    if sin_half < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    # identity->q_delta 的球面插值等价于按比例缩放轴角半角。
    cos_half = float(np.clip(q[3], -1.0, 1.0))
    angle = 2.0 * math.atan2(sin_half, cos_half)
    scaled_half = float(t) * angle * 0.5
    axis = vector / sin_half
    scaled = np.array(
        [
            axis[0] * math.sin(scaled_half),
            axis[1] * math.sin(scaled_half),
            axis[2] * math.sin(scaled_half),
            math.cos(scaled_half),
        ],
        dtype=float,
    )
    return _normalize_quat(scaled)


def vr_delta_to_target_tcp(
    vr_pos,
    vr_quat,
    ref_vr_pos,
    ref_vr_quat,
    ref_tcp_pos,
    ref_tcp_quat,
    vr_axis_map,
    position_scale,
    rotation_scale,
):
    """复刻 ar5_arm_gateway MODE_POSE 的相对映射数学."""
    current_vr_pos = np.asarray(vr_pos, dtype=float)
    current_vr_quat = _normalize_quat(vr_quat)
    reference_vr_pos = np.asarray(ref_vr_pos, dtype=float)
    reference_vr_quat = _normalize_quat(ref_vr_quat)
    reference_tcp_pos = np.asarray(ref_tcp_pos, dtype=float)
    reference_tcp_quat = _normalize_quat(ref_tcp_quat)
    position_map, rotation_map = vr_axis_maps(vr_axis_map)

    # 平移先按 position_scale 缩放，再由 vr_rot 从 VR 轴映射到机器人 base 轴。
    position_delta = (current_vr_pos - reference_vr_pos) * float(position_scale)

    # 旋转增量 q_current * inverse(q_ref)，缩放后叠加到锁定瞬间 TCP 姿态。
    q_delta = _quat_multiply(current_vr_quat, _quat_inverse(reference_vr_quat))
    scaled = _slerp_from_identity(q_delta, rotation_scale)
    mapped_rotation = rotation_map @ _quat_to_matrix(scaled) @ rotation_map.T
    mapped_position = position_map @ position_delta

    target_pos = reference_tcp_pos + mapped_position
    target_quat = _quat_multiply(_matrix_to_quat(mapped_rotation), reference_tcp_quat)
    return target_pos, _normalize_quat(target_quat)


def clamp_joints(joints, joint_limits_min, joint_limits_max):
    if len(joints) != 7:
        return [0.0] * 7
    # 关节角度限制
    clamped = []
    for value, lower, upper in zip(joints, joint_limits_min, joint_limits_max):
        clamped.append(min(max(float(value), float(lower)), float(upper)))
    return clamped
