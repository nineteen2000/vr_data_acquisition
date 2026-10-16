#!/usr/bin/env python3
"""Extract one ROS 2 bag episode into staging for LeRobot conversion.

Run with system ROS Python 3.10 after sourcing the ROS workspace. Inputs are one
rosbag directory and the configured topics in excute_plan.md. Outputs go under
/home/lc/datasets/ar5_lhandpro_v1_staging/<episode>/ by default.

Topic semantics: observation.state uses measured /left_arm/joint_states plus
/hand_state. action uses /left_arm/debug_servo_sent_target and
/hand_applied_command when available; older smoke bags may contain only
/left_arm/debug_servo_target and /hand_command. All alignment uses bag
timestamps; header stamps are saved for diagnostics only.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag_lerobot_common import (
    HAND_NAMES,
    IMAGE_TOPICS,
    JOINT_NAMES,
    append_jsonl,
    expand_hand_positions,
    header_stamp_to_ns,
    save_stream_npz,
    write_json,
)


STREAM_TOPICS = {
    "/left_arm/joint_states": ("joint_states", JOINT_NAMES),
    "/left_arm/debug_servo_sent_target": ("debug_servo_sent_target", JOINT_NAMES),
    "/left_arm/debug_servo_target": ("debug_servo_target", JOINT_NAMES),
    "/left_arm/debug_raw_ik_target": ("debug_raw_ik_target", JOINT_NAMES),
    "/left_arm/debug_servo_error": ("debug_servo_error", JOINT_NAMES),
    "/hand_state": ("hand_state", HAND_NAMES),
    "/hand_applied_command": ("hand_applied_command", HAND_NAMES),
    "/hand_command": ("hand_command", HAND_NAMES),
}

EVENT_TOPICS = {
    "/left_arm/status": "left_arm_status",
    "/rosout": "rosout",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("/home/lc/datasets/ar5_lhandpro_v1_staging"),
        help="staging root directory",
    )
    parser.add_argument("--episode-id", help="episode directory name; defaults to bag directory name")
    parser.add_argument("--task", default="act_data_collect", help="task string saved with episode metadata")
    return parser.parse_args()


def make_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def joint_state_record(msg, bag_time_ns: int) -> dict | None:
    values = list(msg.position)
    if len(values) < 7:
        return None
    return {
        "bag_time_ns": bag_time_ns,
        "header_time_ns": header_stamp_to_ns(msg),
        "values": [float(v) for v in values[:7]],
    }


def hand_state_record(msg, bag_time_ns: int) -> dict | None:
    values = list(msg.motor_positions)
    if len(values) < 6:
        return None
    return {
        "bag_time_ns": bag_time_ns,
        "header_time_ns": header_stamp_to_ns(msg),
        "values": [float(v) for v in values[:6]],
    }


def hand_command_record(msg, bag_time_ns: int) -> dict | None:
    values = expand_hand_positions(msg.preset_name, list(msg.motor_positions))
    if values is None:
        return None
    return {
        "bag_time_ns": bag_time_ns,
        "header_time_ns": header_stamp_to_ns(msg),
        "values": [float(v) for v in values],
        "preset_name": msg.preset_name,
        "enabled": bool(msg.enabled),
    }


def write_image(bridge: CvBridge, msg, episode_dir: Path, camera_name: str, frame_index: int, bag_time_ns: int) -> None:
    rgb = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    rgb = np.asarray(rgb, dtype=np.uint8)
    image_dir = episode_dir / "images" / camera_name
    image_dir.mkdir(parents=True, exist_ok=True)
    rel_path = Path("images") / camera_name / f"{frame_index:06d}.png"
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(episode_dir / rel_path), bgr):
        raise RuntimeError(f"failed to write image {episode_dir / rel_path}")
    append_jsonl(
        image_dir / "index.jsonl",
        {
            "bag_time_ns": bag_time_ns,
            "header_time_ns": header_stamp_to_ns(msg),
            "path": rel_path.as_posix(),
            "encoding": msg.encoding,
            "height": int(msg.height),
            "width": int(msg.width),
        },
    )


def frequency_report(times: list[int]) -> dict:
    if len(times) < 2:
        return {"count": len(times), "hz_median": None}
    deltas = np.diff(np.asarray(sorted(times), dtype=np.float64)) / 1_000_000_000.0
    deltas = deltas[deltas > 0]
    if len(deltas) == 0:
        return {"count": len(times), "hz_median": None}
    return {"count": len(times), "hz_median": float(1.0 / np.median(deltas))}


def main() -> None:
    args = parse_args()
    episode_id = args.episode_id or args.bag.name
    episode_dir = args.output_root / episode_id
    if episode_dir.exists():
        raise FileExistsError(f"staging episode already exists: {episode_dir}")
    episode_dir.mkdir(parents=True)

    reader = make_reader(args.bag)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}
    bridge = CvBridge()

    streams: dict[str, list[dict]] = defaultdict(list)
    counts: dict[str, list[int]] = defaultdict(list)
    image_counts: dict[str, int] = defaultdict(int)
    invalid_counts: dict[str, int] = defaultdict(int)

    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        counts[topic].append(int(bag_time_ns))

        if topic in IMAGE_TOPICS:
            camera_name = IMAGE_TOPICS[topic]
            write_image(bridge, msg, episode_dir, camera_name, image_counts[camera_name], int(bag_time_ns))
            image_counts[camera_name] += 1
            continue

        if topic in STREAM_TOPICS:
            stream_name, _ = STREAM_TOPICS[topic]
            if stream_name.startswith("hand_") and stream_name != "hand_state":
                record = hand_command_record(msg, int(bag_time_ns))
            elif stream_name == "hand_state":
                record = hand_state_record(msg, int(bag_time_ns))
            else:
                record = joint_state_record(msg, int(bag_time_ns))
            if record is None:
                invalid_counts[stream_name] += 1
            else:
                streams[stream_name].append(record)
            continue

        if topic in EVENT_TOPICS:
            event_name = EVENT_TOPICS[topic]
            text = getattr(msg, "data", getattr(msg, "msg", ""))
            append_jsonl(
                episode_dir / "events" / f"{event_name}.jsonl",
                {
                    "bag_time_ns": int(bag_time_ns),
                    "header_time_ns": header_stamp_to_ns(msg),
                    "text": str(text),
                },
            )

    for _, (stream_name, names) in STREAM_TOPICS.items():
        save_stream_npz(episode_dir / "streams" / f"{stream_name}.npz", streams.get(stream_name, []), names)

    report = {
        "episode_id": episode_id,
        "bag": str(args.bag),
        "task": args.task,
        "topics": {topic: frequency_report(times) for topic, times in sorted(counts.items())},
        "image_counts": dict(image_counts),
        "stream_counts": {name: len(records) for name, records in sorted(streams.items())},
        "invalid_counts": dict(invalid_counts),
        "timestamp_policy": "bag_timestamp_for_alignment_header_stamp_for_diagnostics",
    }
    write_json(episode_dir / "episode_report.json", report)
    write_json(episode_dir / "episode_meta.json", {"episode_id": episode_id, "task": args.task})
    print(f"wrote staging episode: {episode_dir}")


if __name__ == "__main__":
    main()
