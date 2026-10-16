import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from rosbag_lerobot_common import (  # noqa: E402
    IMAGE_KEYS,
    build_frame_plan,
    choose_action_sources,
    expand_hand_positions,
)


def _ns(seconds):
    return int(seconds * 1_000_000_000)


def _images(times):
    return {
        name: [{"bag_time_ns": _ns(t), "path": f"{name}_{i}.png"} for i, t in enumerate(times)]
        for name in IMAGE_KEYS
    }


def _stream(times, width, value):
    return {
        "t": np.asarray(times, dtype=np.float64),
        "bag_time_ns": np.asarray([_ns(t) for t in times], dtype=np.int64),
        "values": np.full((len(times), width), value, dtype=np.float32),
    }


def test_expand_hand_positions_uses_presets_and_rejects_empty_intent():
    assert expand_hand_positions("middle", []) == [3000] * 6
    assert expand_hand_positions("", [1, 2, 3, 4, 5, 6]) == [1, 2, 3, 4, 5, 6]
    assert expand_hand_positions("", []) is None
    assert expand_hand_positions("bad", []) is None


def test_formal_mode_requires_sent_arm_and_applied_hand_sources():
    streams = {
        "debug_servo_target": _stream([0.0], 7, 0.1),
        "hand_command": _stream([0.0], 6, 0.0),
    }
    with pytest.raises(ValueError, match="debug_servo_sent_target"):
        choose_action_sources(streams, "formal")


def test_smoke_mode_falls_back_to_development_sources():
    streams = {
        "debug_servo_target": _stream([0.0], 7, 0.1),
        "hand_command": _stream([0.0], 6, 0.0),
    }
    arm_source, hand_source, quality = choose_action_sources(streams, "smoke")

    assert arm_source == "debug_servo_target"
    assert hand_source == "hand_command"
    assert quality["data_quality"] == "development_not_formal"
    assert quality["hand_action_source"] == "upstream_intent"


def test_build_frame_plan_keeps_hand_action_causal_until_next_event():
    streams = {
        "joint_states": _stream([0.0, 0.1, 0.2], 7, 0.2),
        "hand_state": _stream([0.0, 0.1, 0.2], 6, 10.0),
        "debug_servo_sent_target": _stream([0.0, 0.1, 0.2], 7, 0.3),
        "hand_applied_command": {
            "t": np.asarray([0.0, 0.2], dtype=np.float64),
            "bag_time_ns": np.asarray([_ns(0.0), _ns(0.2)], dtype=np.int64),
            "values": np.asarray([[0] * 6, [6000] * 6], dtype=np.float32),
        },
    }

    frames, report = build_frame_plan(_images([0.0, 0.1, 0.2]), streams, "formal")

    assert report["data_quality"] == "formal"
    assert report["kept_frames"] == 3
    assert frames[0]["action"][7:].tolist() == [0.0] * 6
    assert frames[1]["action"][7:].tolist() == [0.0] * 6
    assert frames[2]["action"][7:].tolist() == [6000.0] * 6


def test_build_frame_plan_drops_joint_interpolation_gap_over_limit():
    streams = {
        "joint_states": _stream([0.0, 0.2], 7, 0.2),
        "hand_state": _stream([0.0, 0.1, 0.2], 6, 10.0),
        "debug_servo_sent_target": _stream([0.0, 0.1, 0.2], 7, 0.3),
        "hand_applied_command": _stream([0.0], 6, 0.0),
    }

    frames, report = build_frame_plan(_images([0.0, 0.1, 0.2]), streams, "formal")

    assert len(frames) == 2
    assert report["dropped"]["joint_state_interp"] == 1
