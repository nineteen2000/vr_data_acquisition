"""
正运动学、几何雅可比、阻尼最小二乘数值 IK。
基于 robot_model.py 的 V2 URDF 几何（joint origin/axis + 法兰偏移）实现：
- forward_kinematics：7 轴关节角 -> 法兰(tcp) 在 base 系的位置与四元数；
- jacobian：base 系几何雅可比（前 3 行线速度、后 3 行角速度）；
- solve_ik：阻尼最小二乘迭代，由目标法兰位姿求 7 轴关节角，需 seed 起步并按关节限位 clamp。
单位：长度 m、角度 rad、四元数 (x, y, z, w)。解析动力学接口仍留作后续。
"""

import math

import numpy as np

from teleop_core.robot_model import (
    FLANGE_ORIGIN_RPY,
    FLANGE_ORIGIN_XYZ,
    JOINT_AXES,
    JOINT_ORIGINS_RPY,
    JOINT_ORIGINS_XYZ,
)


_DLS_DAMPING = 0.05
_SMALL_ANGLE = 1e-12


# 检查关节数组形状并转为浮点 numpy 数组
def _as_joint_array(joints):
    values = np.asarray(joints, dtype=float)
    if values.shape != (7,):
        raise ValueError("joints must be a length-7 sequence")
    return values


# 检查三维向量形状并转为浮点 numpy 数组
def _as_vec3(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} must be a length-3 sequence")
    return vector


# 检查四元数形状并归一化
def _normalize_quat(quat):
    values = np.asarray(quat, dtype=float)
    if values.shape != (4,):
        raise ValueError("quat must be a length-4 sequence")
    norm = np.linalg.norm(values)
    if norm <= _SMALL_ANGLE:
        raise ValueError("quat norm must be non-zero")
    return values / norm


# 生成平移齐次变换矩阵
def _translation_matrix(xyz):
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


# 生成旋转齐次变换矩阵
def _rotation_matrix_to_transform(rotation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    return transform


# URDF rpy 转旋转矩阵，固定轴 X(roll) -> Y(pitch) -> Z(yaw)
def _rpy_to_matrix(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=float,
    )
    ry = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )
    rz = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rz @ ry @ rx


# 罗德里格斯公式：绕单位轴 axis 旋转 theta
def _axis_angle_to_matrix(axis, theta):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(float(theta))
    s = math.sin(float(theta))
    v = 1.0 - c

    return np.array(
        [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ],
        dtype=float,
    )


# 旋转矩阵转四元数，返回顺序为 (x, y, z, w)
def _matrix_to_quat(rotation):
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))

    # 分支法避免 trace 或主对角元素接近零时除零。
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + r[1, 1] - r[0, 0] - r[2, 2])) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + r[2, 2] - r[0, 0] - r[1, 1])) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s

    return _normalize_quat(np.array([qx, qy, qz, qw], dtype=float))


# 四元数乘法，输入输出顺序均为 (x, y, z, w)
def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = _normalize_quat(q1)
    x2, y2, z2, w2 = _normalize_quat(q2)

    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


# 单位四元数求逆，顺序为 (x, y, z, w)
def _quat_inverse(quat):
    q = np.asarray(quat, dtype=float)
    norm_sq = float(np.dot(q, q))
    if norm_sq <= _SMALL_ANGLE:
        raise ValueError("quat norm must be non-zero")
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float) / norm_sq


# 四元数转旋转向量，取 w >= 0 的最短路径
def _quat_to_rotvec(quat):
    q = _normalize_quat(quat)
    if q[3] < 0.0:
        q = -q

    vector = q[:3]
    sin_half = np.linalg.norm(vector)
    if sin_half < _SMALL_ANGLE:
        # 小角度下 angle/sin(angle/2) -> 2，直接用线性近似。
        return 2.0 * vector

    angle = 2.0 * math.atan2(sin_half, q[3])
    return vector / sin_half * angle


# 生成一个关节的固定 origin 变换
def _fixed_origin_transform(xyz, rpy):
    return _translation_matrix(xyz) @ _rotation_matrix_to_transform(_rpy_to_matrix(rpy))


# 正运动学：由 7 轴关节角计算法兰(tcp)在 base 系下的位置和四元数
def forward_kinematics(joints):
    q = _as_joint_array(joints)
    transform = np.eye(4, dtype=float)

    # 按 URDF 顺序累乘每个 revolute joint 的固定 origin 与关节旋转。
    for index, angle in enumerate(q):
        fixed = _fixed_origin_transform(JOINT_ORIGINS_XYZ[index], JOINT_ORIGINS_RPY[index])
        rotation = _rotation_matrix_to_transform(
            _axis_angle_to_matrix(JOINT_AXES[index], angle)
        )
        transform = transform @ fixed @ rotation

    # FK 终点定义为法兰(tcp)，因此 link7 后继续乘 fixed flan_joint。
    flange = _fixed_origin_transform(FLANGE_ORIGIN_XYZ, FLANGE_ORIGIN_RPY)
    transform = transform @ flange

    position = transform[:3, 3].copy()
    quat = _matrix_to_quat(transform[:3, :3])
    return position, quat


# 几何雅可比：前 3 行线速度，后 3 行角速度，均表达在 base 系
def jacobian(joints):
    q = _as_joint_array(joints)
    transform = np.eye(4, dtype=float)
    origins = []
    axes = []

    # 缓存每个关节固定 origin/rpy 后、关节自身旋转前的轴线位置和方向。
    for index, angle in enumerate(q):
        fixed = _fixed_origin_transform(JOINT_ORIGINS_XYZ[index], JOINT_ORIGINS_RPY[index])
        transform = transform @ fixed
        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, :3] @ np.asarray(JOINT_AXES[index], dtype=float))

        rotation = _rotation_matrix_to_transform(
            _axis_angle_to_matrix(JOINT_AXES[index], angle)
        )
        transform = transform @ rotation

    # 末端点使用法兰(tcp)原点。
    flange = _fixed_origin_transform(FLANGE_ORIGIN_XYZ, FLANGE_ORIGIN_RPY)
    end_position = (transform @ flange)[:3, 3]

    result = np.zeros((6, 7), dtype=float)
    for index, (origin, axis) in enumerate(zip(origins, axes)):
        axis = axis / np.linalg.norm(axis)
        result[:3, index] = np.cross(axis, end_position - origin)
        result[3:, index] = axis
    return result


# 阻尼最小二乘 IK：由目标法兰(tcp)位姿迭代求 7 轴关节角
def solve_ik(
    target_position,
    target_quat,
    seed_joints,
    *,
    joint_limits_min,
    joint_limits_max,
    max_iters=100,
    pos_tol=1e-4,
    rot_tol=1e-3,
):
    target_position = _as_vec3(target_position, "target_position")
    target_quat = _normalize_quat(target_quat)
    q = _as_joint_array(seed_joints).copy()
    lower = _as_joint_array(joint_limits_min)
    upper = _as_joint_array(joint_limits_max)
    q = np.clip(q, lower, upper)

    # λ 越大越稳但收敛越慢；λ²I 让近奇异处的 6x6 系统保持可解。
    damping_matrix = (_DLS_DAMPING * _DLS_DAMPING) * np.eye(6, dtype=float)

    for iteration in range(1, int(max_iters) + 1):
        current_position, current_quat = forward_kinematics(q)
        position_error = target_position - current_position
        quat_error = _quat_multiply(target_quat, _quat_inverse(current_quat))
        rotation_error = _quat_to_rotvec(quat_error)

        if (
            np.linalg.norm(position_error) < pos_tol
            and np.linalg.norm(rotation_error) < rot_tol
        ):
            return q, True, iteration

        error = np.concatenate([position_error, rotation_error])
        j = jacobian(q)
        lhs = j @ j.T + damping_matrix
        dq = j.T @ np.linalg.solve(lhs, error)
        q = np.clip(q + dq, lower, upper)

    return q, False, int(max_iters)
