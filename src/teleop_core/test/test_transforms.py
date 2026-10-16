import math
from types import SimpleNamespace

import pytest

from teleop_core.transforms import (
    clamp_joints,
    finite_pose,
    quaternion_to_euler,
    vr_delta_to_target_tcp,
    scaled_pose,
    shortest_angle,
)


def pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_quaternion_to_euler_yaw_90deg():
    half = math.sqrt(0.5)
    _, _, yaw = quaternion_to_euler(SimpleNamespace(x=0.0, y=0.0, z=half, w=half))

    assert yaw == pytest.approx(math.pi / 2.0, abs=1e-9)


def test_shortest_angle_wraps_to_pi_range():
    assert shortest_angle(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert shortest_angle(-3.0 * math.pi) == pytest.approx(-math.pi, abs=1e-9)


def test_scaled_pose_keeps_reference_and_scales_delta():
    reference = pose(x=1.0, y=2.0, z=3.0)
    current = pose(x=2.0, y=4.0, z=6.0)

    scaled = scaled_pose(current, reference, 0.5)

    assert scaled.position.x == pytest.approx(1.5, abs=1e-9)
    assert scaled.position.y == pytest.approx(3.0, abs=1e-9)
    assert scaled.position.z == pytest.approx(4.5, abs=1e-9)


def test_finite_pose_rejects_nan():
    assert finite_pose(pose())
    assert not finite_pose(pose(x=math.nan))


def test_vr_delta_to_target_tcp_uses_4x4_position_sign_column():
    half = math.sqrt(0.5)
    axis_map = [
        1.0, 0.0, 0.0, -1.0,
        0.0, 1.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]

    target_pos, target_quat = vr_delta_to_target_tcp(
        [1.0, 0.0, 0.0],
        [0.0, 0.0, half, half],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        axis_map,
        1.0,
        1.0,
    )

    assert target_pos == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)
    assert target_quat == pytest.approx([0.0, 0.0, half, half], abs=1e-9)


def test_clamp_joints_uses_urdf_limits_shape():
    result = clamp_joints(
        [-10.0, 0.0, 10.0, 3.0, 0.0, 2.0, -2.0],
        [-3.1067, -2.0944, -3.1067, -1.0472, -3.1067, -0.9599, -0.9599],
        [3.1067, 2.0944, 3.1067, 2.5307, 3.1067, 0.9599, 0.9599],
    )

    assert result == pytest.approx(
        [-3.1067, 0.0, 3.1067, 2.5307, 0.0, 0.9599, -0.9599],
        abs=1e-9,
    )
