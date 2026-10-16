import numpy as np
import pytest

from teleop_core.kinematics import (
    _quat_inverse,
    _quat_multiply,
    _quat_to_rotvec,
    forward_kinematics,
    jacobian,
    solve_ik,
)
from teleop_core.robot_model import JOINT_LIMITS_MAX, JOINT_LIMITS_MIN


JOINT_LIMITS_MIN_ARRAY = np.asarray(JOINT_LIMITS_MIN, dtype=float)
JOINT_LIMITS_MAX_ARRAY = np.asarray(JOINT_LIMITS_MAX, dtype=float)


# 计算两个四元数之间的最短旋转向量误差
def _rotation_error(target_quat, current_quat):
    quat_error = _quat_multiply(target_quat, _quat_inverse(current_quat))
    return _quat_to_rotvec(quat_error)


# FK 零位应落在 base 正上方，并包含 link7 到法兰的 0.09 m 偏移
def test_forward_kinematics_zero_pose_matches_urdf_geometry():
    position, quat = forward_kinematics([0.0] * 7)

    assert position == pytest.approx([0.0, 0.0, 0.8745], abs=1e-6)
    assert np.linalg.norm(quat) == pytest.approx(1.0, abs=1e-9)


# IK 对若干合法构型做 FK -> IK -> FK 闭环，验证法兰位姿能收敛回目标
@pytest.mark.parametrize(
    "joints",
    [
        [0.2, -0.4, 0.3, 0.5, -0.2, 0.25, -0.15],
        [-0.35, 0.25, -0.45, 0.7, 0.4, -0.2, 0.2],
        [0.45, -0.3, 0.25, 0.4, -0.35, 0.15, 0.1],
    ],
)
def test_solve_ik_round_trip_from_nearby_seed(joints):
    target_joints = np.asarray(joints, dtype=float)
    target_position, target_quat = forward_kinematics(target_joints)
    seed = target_joints + np.asarray([0.02, -0.03, 0.015, -0.02, 0.025, -0.015, 0.01])

    solved, converged, iters = solve_ik(
        target_position,
        target_quat,
        seed,
        joint_limits_min=JOINT_LIMITS_MIN,
        joint_limits_max=JOINT_LIMITS_MAX,
        max_iters=200,
    )
    solved_position, solved_quat = forward_kinematics(solved)
    rotation_error = _rotation_error(target_quat, solved_quat)

    assert converged
    assert iters <= 20
    assert np.linalg.norm(target_position - solved_position) < 1e-4
    assert np.linalg.norm(rotation_error) < 1e-3


# 相近目标沿用相近 seed 时，冗余 7 轴 IK 解应保持连续不跳变
def test_solve_ik_keeps_seed_continuity_for_nearby_targets():
    joints_a = np.asarray([0.2, -0.35, 0.25, 0.55, -0.2, 0.2, -0.1], dtype=float)
    joints_b = joints_a + np.asarray([0.005, -0.004, 0.003, -0.006, 0.004, -0.003, 0.002])
    position_a, quat_a = forward_kinematics(joints_a)
    position_b, quat_b = forward_kinematics(joints_b)

    solved_a, converged_a, _ = solve_ik(
        position_a,
        quat_a,
        joints_a + 0.02,
        joint_limits_min=JOINT_LIMITS_MIN,
        joint_limits_max=JOINT_LIMITS_MAX,
        max_iters=200,
    )
    solved_b, converged_b, _ = solve_ik(
        position_b,
        quat_b,
        solved_a,
        joint_limits_min=JOINT_LIMITS_MIN,
        joint_limits_max=JOINT_LIMITS_MAX,
        max_iters=200,
    )

    assert converged_a
    assert converged_b
    assert np.linalg.norm(solved_b - solved_a) < 0.1


# 不可达目标也必须在每次 DLS 更新后被 clamp 到 URDF 关节限位内
def test_solve_ik_clamps_to_joint_limits():
    target_position = np.asarray([10.0, -10.0, 10.0], dtype=float)
    target_quat = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    seed = np.asarray([10.0, -10.0, 10.0, 10.0, -10.0, 10.0, -10.0], dtype=float)

    solved, _, _ = solve_ik(
        target_position,
        target_quat,
        seed,
        joint_limits_min=JOINT_LIMITS_MIN,
        joint_limits_max=JOINT_LIMITS_MAX,
        max_iters=10,
    )

    assert np.all(solved >= JOINT_LIMITS_MIN_ARRAY - 1e-12)
    assert np.all(solved <= JOINT_LIMITS_MAX_ARRAY + 1e-12)


# 用中心差分校验几何雅可比前 3 行的线速度部分
def test_jacobian_linear_part_matches_center_difference():
    joints = np.asarray([0.25, -0.35, 0.4, 0.6, -0.3, 0.2, -0.15], dtype=float)
    analytical = jacobian(joints)
    epsilon = 1e-6
    numerical = np.zeros((3, 7), dtype=float)

    for index in range(7):
        delta = np.zeros(7, dtype=float)
        delta[index] = epsilon
        position_plus, _ = forward_kinematics(joints + delta)
        position_minus, _ = forward_kinematics(joints - delta)
        numerical[:, index] = (position_plus - position_minus) / (2.0 * epsilon)

    assert analytical[:3, :] == pytest.approx(numerical, abs=1e-4, rel=1e-4)


# 用中心差分校验几何雅可比后 3 行的角速度部分
def test_jacobian_angular_part_matches_center_difference():
    joints = np.asarray([0.25, -0.35, 0.4, 0.6, -0.3, 0.2, -0.15], dtype=float)
    analytical = jacobian(joints)
    epsilon = 1e-6
    numerical = np.zeros((3, 7), dtype=float)

    for index in range(7):
        delta = np.zeros(7, dtype=float)
        delta[index] = epsilon
        _, quat_plus = forward_kinematics(joints + delta)
        _, quat_minus = forward_kinematics(joints - delta)
        # 姿态增量转旋转向量再除以 2ε，即该关节对末端角速度的贡献
        rotation_delta = _quat_to_rotvec(
            _quat_multiply(quat_plus, _quat_inverse(quat_minus))
        )
        numerical[:, index] = rotation_delta / (2.0 * epsilon)

    assert analytical[3:, :] == pytest.approx(numerical, abs=1e-4, rel=1e-4)
