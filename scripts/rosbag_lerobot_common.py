#!/usr/bin/env python3
"""Shared helpers for AR5 + LHandPro rosbag to LeRobot conversion.

The conversion uses ROS bag timestamps as the v1 alignment clock. Header stamps
are kept only for diagnostics. State/action vectors are ordered as
joint1..joint7, hand_motor1..hand_motor6. Arm values are radians; hand values
are LHandPro motor positions.
"""

from __future__ import annotations

import json
from bisect import bisect_left, bisect_right
from pathlib import Path
from typing import Any

import numpy as np


FPS = 10
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_MAX_DT_S = 0.080
JOINT_BRACKET_MAX_DT_S = 0.100
HAND_STATE_MAX_DT_S = 0.150
ARM_ACTION_MAX_AGE_S = 0.100

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
HAND_NAMES = [f"hand_motor{i}" for i in range(1, 7)]
STATE_ACTION_NAMES = JOINT_NAMES + HAND_NAMES

IMAGE_KEYS = {
    "wrist_left": "observation.images.wrist_left",
    "wrist_right": "observation.images.wrist_right",
    "global": "observation.images.global",
}

IMAGE_TOPICS = {
    "/wrist_camera/left_color/image_raw": "wrist_left",
    "/wrist_camera/right_color/image_raw": "wrist_right",
    "/global_camera/color/image_raw": "global",
}

PRESETS = {
    "open": [0, 0, 0, 0, 0, 0],
    "middle": [3000, 3000, 3000, 3000, 3000, 3000],
    "close": [6000, 6000, 6000, 6000, 6000, 6000],
}

FEATURES = {
    "observation.images.wrist_left": {
        "dtype": "video",
        "shape": (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        "names": ["height", "width", "channels"],
        "info": {"is_depth_map": False},
    },
    "observation.images.wrist_right": {
        "dtype": "video",
        "shape": (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        "names": ["height", "width", "channels"],
        "info": {"is_depth_map": False},
    },
    "observation.images.global": {
        "dtype": "video",
        "shape": (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
        "names": ["height", "width", "channels"],
        "info": {"is_depth_map": False},
    },
    "observation.state": {
        "dtype": "float32",
        "shape": (13,),
        "names": STATE_ACTION_NAMES,
    },
    "action": {
        "dtype": "float32",
        "shape": (13,),
        "names": STATE_ACTION_NAMES,
    },
}


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_stamp_to_ns(msg: Any) -> int:
    if not hasattr(msg, "header"):
        return -1
    stamp = getattr(msg.header, "stamp", None)
    if stamp is None:
        return -1
    value = stamp_to_ns(stamp)
    return value if value > 0 else -1


def ns_to_s(ns: int | np.integer) -> float:
    return float(ns) / 1_000_000_000.0


def expand_hand_positions(preset_name: str, motor_positions: list[int] | tuple[int, ...]) -> list[int] | None:
    if preset_name:
        return list(PRESETS[preset_name]) if preset_name in PRESETS else None
    if len(motor_positions) == 6:
        return [int(v) for v in motor_positions]
    return None


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def save_stream_npz(path: Path, records: list[dict[str, Any]], names: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if records:
        values = np.asarray([record["values"] for record in records], dtype=np.float32)
        bag_time_ns = np.asarray([record["bag_time_ns"] for record in records], dtype=np.int64)
        header_time_ns = np.asarray([record.get("header_time_ns", -1) for record in records], dtype=np.int64)
        preset_names = np.asarray([record.get("preset_name", "") for record in records], dtype=object)
        enabled = np.asarray([record.get("enabled", True) for record in records], dtype=bool)
    else:
        width = len(names or [])
        values = np.empty((0, width), dtype=np.float32)
        bag_time_ns = np.empty((0,), dtype=np.int64)
        header_time_ns = np.empty((0,), dtype=np.int64)
        preset_names = np.empty((0,), dtype=object)
        enabled = np.empty((0,), dtype=bool)
    np.savez_compressed(
        path,
        values=values,
        bag_time_ns=bag_time_ns,
        header_time_ns=header_time_ns,
        names=np.asarray(names or [], dtype=object),
        preset_names=preset_names,
        enabled=enabled,
    )


def load_stream_npz(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    data = np.load(path, allow_pickle=True)
    bag_time_ns = data["bag_time_ns"].astype(np.int64)
    return {
        "values": data["values"].astype(np.float32),
        "bag_time_ns": bag_time_ns,
        "t": np.asarray([ns_to_s(v) for v in bag_time_ns], dtype=np.float64),
        "header_time_ns": data["header_time_ns"].astype(np.int64),
        "names": [str(v) for v in data["names"].tolist()],
        "preset_names": [str(v) for v in data["preset_names"].tolist()],
        "enabled": data["enabled"].astype(bool),
    }


def nearest_index(times: np.ndarray, t: float, max_abs_dt: float) -> int | None:
    if len(times) == 0:
        return None
    idx = bisect_left(times, t)
    candidates = []
    if idx < len(times):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    best = min(candidates, key=lambda i: abs(float(times[i]) - t))
    return best if abs(float(times[best]) - t) <= max_abs_dt else None


def previous_index(times: np.ndarray, t: float, max_age: float | None = None) -> int | None:
    idx = bisect_right(times, t) - 1
    if idx < 0:
        return None
    if max_age is not None and t - float(times[idx]) > max_age:
        return None
    return idx


def interpolate_vector(stream: dict[str, Any], t: float, max_bracket_dt: float) -> np.ndarray | None:
    times = stream["t"]
    values = stream["values"]
    if len(times) == 0:
        return None
    right = bisect_left(times, t)
    if right < len(times) and abs(float(times[right]) - t) < 1e-9:
        return values[right]
    left = right - 1
    if left < 0 or right >= len(times):
        return None
    t0 = float(times[left])
    t1 = float(times[right])
    if t1 - t0 > max_bracket_dt or t1 <= t0:
        return None
    alpha = (t - t0) / (t1 - t0)
    return values[left] * (1.0 - alpha) + values[right] * alpha


def uniform_time_grid(start_s: float, end_s: float, fps: int = FPS) -> np.ndarray:
    if end_s < start_s:
        return np.empty((0,), dtype=np.float64)
    count = int(np.floor((end_s - start_s) * fps + 1e-9)) + 1
    return start_s + np.arange(count, dtype=np.float64) / float(fps)


def choose_action_sources(streams: dict[str, dict[str, Any]], mode: str) -> tuple[str, str, dict[str, str]]:
    quality: dict[str, str] = {}
    if "debug_servo_sent_target" in streams:
        arm_source = "debug_servo_sent_target"
    elif mode == "formal":
        raise ValueError("formal build requires /left_arm/debug_servo_sent_target")
    elif "debug_servo_target" in streams:
        arm_source = "debug_servo_target"
        quality["data_quality"] = "development_not_formal"
        quality["arm_action_source"] = "accepted_target_before_sdk_send"
    else:
        raise ValueError("missing arm action stream")

    if "hand_applied_command" in streams:
        hand_source = "hand_applied_command"
        quality["hand_action_source"] = "applied_command"
    elif mode == "formal":
        raise ValueError("formal build requires /hand_applied_command")
    elif "hand_command" in streams:
        hand_source = "hand_command"
        quality["data_quality"] = "development_not_formal"
        quality["hand_action_source"] = "upstream_intent"
    else:
        raise ValueError("missing hand action stream")

    quality.setdefault("data_quality", "formal")
    quality.setdefault("arm_action_source", "sent_target_after_sdk_send")
    return arm_source, hand_source, quality


def validate_state_action(state: np.ndarray, action: np.ndarray) -> str | None:
    if state.shape != (13,):
        return "state_dim"
    if action.shape != (13,):
        return "action_dim"
    if not np.all(np.isfinite(state)) or not np.all(np.isfinite(action)):
        return "non_finite"
    if np.any(action[7:] < 0) or np.any(action[7:] > 10000):
        return "hand_action_range"
    return None


def letterbox_rgb(rgb: np.ndarray, width: int = IMAGE_WIDTH, height: int = IMAGE_HEIGHT) -> np.ndarray:
    import cv2

    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"expected HWC RGB image, got shape={rgb.shape}")
    src_h, src_w = rgb.shape[:2]
    scale = min(width / src_w, height / src_h)
    new_w = max(1, int(round(src_w * scale)))
    new_h = max(1, int(round(src_h * scale)))
    resized = cv2.resize(rgb, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    y0 = (height - new_h) // 2
    x0 = (width - new_w) // 2
    canvas[y0 : y0 + new_h, x0 : x0 + new_w] = resized
    return canvas


def build_frame_plan(
    image_indices: dict[str, list[dict[str, Any]]],
    streams: dict[str, dict[str, Any]],
    mode: str,
    fps: int = FPS,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    arm_source, hand_source, quality = choose_action_sources(streams, mode)
    required_numeric = ["joint_states", "hand_state", arm_source, hand_source]
    missing_images = [name for name in IMAGE_KEYS if not image_indices.get(name)]
    missing_streams = [name for name in required_numeric if name not in streams or len(streams[name]["t"]) == 0]
    if missing_images or missing_streams:
        raise ValueError(f"missing required streams images={missing_images} streams={missing_streams}")

    stream_times = [streams[name]["t"] for name in required_numeric]
    bounded_stream_times = [streams[name]["t"] for name in ("joint_states", "hand_state", arm_source)]
    image_times = [np.asarray([record["bag_time_ns"] for record in image_indices[name]], dtype=np.int64) for name in IMAGE_KEYS]
    image_times_s = [np.asarray([ns_to_s(v) for v in values], dtype=np.float64) for values in image_times]
    start_s = max([float(times[0]) for times in stream_times + image_times_s])
    end_s = min([float(times[-1]) for times in bounded_stream_times + image_times_s])

    frames: list[dict[str, Any]] = []
    dropped: dict[str, int] = {}
    per_camera_times = {
        name: np.asarray([ns_to_s(record["bag_time_ns"]) for record in records], dtype=np.float64)
        for name, records in image_indices.items()
    }
    for t in uniform_time_grid(start_s, end_s, fps=fps):
        image_records: dict[str, dict[str, Any]] = {}
        reason = None
        for camera_name in IMAGE_KEYS:
            idx = nearest_index(per_camera_times[camera_name], t, IMAGE_MAX_DT_S)
            if idx is None:
                reason = f"{camera_name}_image_dt"
                break
            image_records[camera_name] = image_indices[camera_name][idx]
        if reason is not None:
            dropped[reason] = dropped.get(reason, 0) + 1
            continue

        arm_state = interpolate_vector(streams["joint_states"], t, JOINT_BRACKET_MAX_DT_S)
        if arm_state is None or arm_state.shape[0] != 7:
            dropped["joint_state_interp"] = dropped.get("joint_state_interp", 0) + 1
            continue
        hand_state_idx = nearest_index(streams["hand_state"]["t"], t, HAND_STATE_MAX_DT_S)
        if hand_state_idx is None:
            dropped["hand_state_dt"] = dropped.get("hand_state_dt", 0) + 1
            continue
        hand_state = streams["hand_state"]["values"][hand_state_idx]
        arm_action_idx = previous_index(streams[arm_source]["t"], t, ARM_ACTION_MAX_AGE_S)
        if arm_action_idx is None:
            dropped["arm_action_age"] = dropped.get("arm_action_age", 0) + 1
            continue
        hand_action_idx = previous_index(streams[hand_source]["t"], t, None)
        if hand_action_idx is None:
            dropped["hand_action_missing"] = dropped.get("hand_action_missing", 0) + 1
            continue

        state = np.concatenate([arm_state[:7], hand_state[:6]]).astype(np.float32)
        action = np.concatenate([
            streams[arm_source]["values"][arm_action_idx][:7],
            streams[hand_source]["values"][hand_action_idx][:6],
        ]).astype(np.float32)
        bad = validate_state_action(state, action)
        if bad:
            dropped[bad] = dropped.get(bad, 0) + 1
            continue

        frames.append(
            {
                "t": float(t),
                "images": image_records,
                "observation.state": state,
                "action": action,
            }
        )

    report = {
        **quality,
        "fps": fps,
        "start_s": start_s,
        "end_s": end_s,
        "candidate_frames": int(len(uniform_time_grid(start_s, end_s, fps=fps))),
        "kept_frames": len(frames),
        "dropped": dropped,
    }
    return frames, report
