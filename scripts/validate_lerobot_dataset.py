#!/usr/bin/env python3
"""Validate the AR5 + LHandPro LeRobotDataset.

Run with src/lerobot/.venv Python 3.12. The script checks metadata, episode
counts, image feature names, strict 10 Hz timestamps, reloadability, policy
feature CHW image shapes, and 13D state/action vectors.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import dataset_to_policy_features

from rosbag_lerobot_common import FPS, IMAGE_KEYS, STATE_ACTION_NAMES, write_json


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/home/lc/datasets/ar5_lhandpro_v1"))
    parser.add_argument("--repo-id", default="local/ar5_lhandpro_v1")
    parser.add_argument("--report", type=Path, help="optional validation report path")
    return parser.parse_args()


def shape_of(value) -> tuple[int, ...]:
    if hasattr(value, "shape"):
        return tuple(int(v) for v in value.shape)
    return tuple(np.asarray(value).shape)


def assert_feature_metadata(info: dict) -> None:
    features = info["features"]
    for image_key in IMAGE_KEYS.values():
        feature = features[image_key]
        if feature["names"] != ["height", "width", "channels"]:
            raise ValueError(f"{image_key} names must be height,width,channels")
        if tuple(feature["shape"]) != (480, 640, 3):
            raise ValueError(f"{image_key} shape must be (480,640,3)")
    for key in ("observation.state", "action"):
        feature = features[key]
        if tuple(feature["shape"]) != (13,):
            raise ValueError(f"{key} shape must be (13,)")
        if feature["names"] != STATE_ACTION_NAMES:
            raise ValueError(f"{key} names mismatch")


def assert_timestamps(dataset: LeRobotDataset) -> None:
    timestamps = np.asarray(dataset.hf_dataset["timestamp"], dtype=np.float64)
    episode_indices = np.asarray(dataset.hf_dataset["episode_index"], dtype=np.int64)
    expected_dt = 1.0 / FPS
    for episode_index in sorted(set(episode_indices.tolist())):
        ep_ts = timestamps[episode_indices == episode_index]
        if len(ep_ts) <= 1:
            continue
        if not np.allclose(np.diff(ep_ts), expected_dt, atol=1e-4):
            raise ValueError(f"episode {episode_index} timestamps are not strict {FPS} Hz")


def main() -> None:
    args = parse_args()
    info_path = args.root / "meta" / "info.json"
    if not info_path.exists():
        raise FileNotFoundError(info_path)
    info = json.loads(info_path.read_text(encoding="utf-8"))
    assert_feature_metadata(info)

    dataset = LeRobotDataset(args.repo_id, root=args.root, return_uint8=True)
    if len(dataset) == 0:
        raise ValueError("dataset has zero frames")
    assert_timestamps(dataset)

    policy_features = dataset_to_policy_features(dataset.meta.features)
    for image_key in IMAGE_KEYS.values():
        if tuple(policy_features[image_key].shape) != (3, 480, 640):
            raise ValueError(f"{image_key} policy shape is not CHW 3x480x640")
    if tuple(policy_features["observation.state"].shape) != (13,):
        raise ValueError("observation.state policy shape mismatch")
    if tuple(policy_features["action"].shape) != (13,):
        raise ValueError("action policy shape mismatch")

    sample = dataset[0]
    sample_shapes = {key: shape_of(sample[key]) for key in IMAGE_KEYS.values()}
    sample_shapes["observation.state"] = shape_of(sample["observation.state"])
    sample_shapes["action"] = shape_of(sample["action"])
    for image_key in IMAGE_KEYS.values():
        if sample_shapes[image_key] != (3, 480, 640):
            raise ValueError(f"{image_key} sample shape mismatch: {sample_shapes[image_key]}")
    if sample_shapes["observation.state"] != (13,) or sample_shapes["action"] != (13,):
        raise ValueError(f"state/action sample shapes mismatch: {sample_shapes}")

    report = {
        "root": str(args.root),
        "repo_id": args.repo_id,
        "num_frames": len(dataset),
        "num_episodes": dataset.num_episodes,
        "sample_shapes": {key: list(value) for key, value in sample_shapes.items()},
        "status": "ok",
    }
    if args.report:
        write_json(args.report, report)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
