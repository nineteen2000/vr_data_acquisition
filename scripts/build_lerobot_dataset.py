#!/usr/bin/env python3
"""Build the AR5 + LHandPro LeRobotDataset from staging episodes.

Run with src/lerobot/.venv Python 3.12. This script reads staging produced by
extract_rosbag_episode.py, aligns all required streams to a strict 10 Hz grid,
letterboxes RGB to 640x480, and writes a LeRobot dataset. Do not run this in the
ROS 2 Humble Python process.

State/action order is joint1..joint7, hand_motor1..hand_motor6. The v1 alignment
clock is the rosbag reader timestamp. Header stamps remain diagnostics in
staging reports and are not written as LeRobot timestamps; LeRobot add_frame()
creates timestamp=frame_index/fps.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from rosbag_lerobot_common import (
    FEATURES,
    FPS,
    IMAGE_KEYS,
    build_frame_plan,
    letterbox_rgb,
    load_stream_npz,
    read_jsonl,
    write_json,
)


DEFAULT_STAGING_ROOT = Path("/home/lc/datasets/ar5_lhandpro_v1_staging")
DEFAULT_FORMAL_ROOT = Path("/home/lc/datasets/ar5_lhandpro_v1")
DEFAULT_SMOKE_ROOT = Path("/home/lc/datasets/ar5_lhandpro_v1_smoke")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staging-root", type=Path, default=DEFAULT_STAGING_ROOT)
    parser.add_argument("--episode", action="append", help="episode id; may be passed multiple times")
    parser.add_argument("--mode", choices=["smoke", "formal"], default="smoke")
    parser.add_argument("--root", type=Path, help="dataset root; defaults by mode")
    parser.add_argument("--repo-id", help="dataset repo id; defaults by mode")
    parser.add_argument("--task", default=None, help="override task string for all frames")
    parser.add_argument("--robot-type", default="xcore_ar5_lhandpro_left")
    return parser.parse_args()


def load_episode_meta(episode_dir: Path, task_override: str | None) -> dict:
    meta_path = episode_dir / "episode_meta.json"
    task = "ar5_lhandpro_task"
    if meta_path.exists():
        import json

        task = json.loads(meta_path.read_text(encoding="utf-8")).get("task", task)
    if task_override:
        task = task_override
    return {"task": task}


def load_episode_indices(episode_dir: Path) -> dict[str, list[dict]]:
    indices: dict[str, list[dict]] = {}
    for camera_name in IMAGE_KEYS:
        records = read_jsonl(episode_dir / "images" / camera_name / "index.jsonl")
        records.sort(key=lambda record: int(record["bag_time_ns"]))
        indices[camera_name] = records
    return indices


def load_episode_streams(episode_dir: Path) -> dict[str, dict]:
    streams: dict[str, dict] = {}
    for path in sorted((episode_dir / "streams").glob("*.npz")):
        loaded = load_stream_npz(path)
        if loaded is not None and len(loaded["t"]) > 0:
            streams[path.stem] = loaded
    return streams


def read_rgb(episode_dir: Path, image_record: dict) -> np.ndarray:
    image_path = episode_dir / image_record["path"]
    bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(f"missing image: {image_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return letterbox_rgb(rgb)


def default_root(mode: str) -> Path:
    return DEFAULT_FORMAL_ROOT if mode == "formal" else DEFAULT_SMOKE_ROOT


def default_repo_id(mode: str) -> str:
    return "local/ar5_lhandpro_v1" if mode == "formal" else "local/ar5_lhandpro_v1_smoke"


def resolve_episodes(staging_root: Path, requested: list[str] | None) -> list[Path]:
    if requested:
        episodes = [staging_root / name for name in requested]
    else:
        episodes = sorted(path for path in staging_root.iterdir() if path.is_dir())
    missing = [str(path) for path in episodes if not path.is_dir()]
    if missing:
        raise FileNotFoundError(f"missing staging episodes: {missing}")
    if not episodes:
        raise ValueError(f"no staging episodes found in {staging_root}")
    return episodes


def main() -> None:
    args = parse_args()
    root = args.root or default_root(args.mode)
    repo_id = args.repo_id or default_repo_id(args.mode)
    if root.exists():
        raise FileExistsError(f"dataset root already exists: {root}")

    episode_dirs = resolve_episodes(args.staging_root, args.episode)
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=FPS,
        features=FEATURES,
        root=root,
        robot_type=args.robot_type,
        use_videos=True,
    )

    reports = []
    try:
        for episode_dir in episode_dirs:
            meta = load_episode_meta(episode_dir, args.task)
            image_indices = load_episode_indices(episode_dir)
            streams = load_episode_streams(episode_dir)
            frames, report = build_frame_plan(image_indices, streams, args.mode, fps=FPS)
            if not frames:
                raise ValueError(f"episode produced zero usable frames: {episode_dir}")
            for frame in frames:
                dataset.add_frame(
                    {
                        "observation.images.wrist_left": read_rgb(episode_dir, frame["images"]["wrist_left"]),
                        "observation.images.wrist_right": read_rgb(episode_dir, frame["images"]["wrist_right"]),
                        "observation.images.global": read_rgb(episode_dir, frame["images"]["global"]),
                        "observation.state": frame["observation.state"].astype(np.float32),
                        "action": frame["action"].astype(np.float32),
                        "task": meta["task"],
                    }
                )
            dataset.save_episode()
            reports.append({"episode": episode_dir.name, **report})
        dataset.finalize()
    except Exception:
        dataset.finalize()
        raise

    write_json(
        root / "conversion_report.json",
        {
            "repo_id": repo_id,
            "root": str(root),
            "mode": args.mode,
            "episodes": reports,
        },
    )
    print(f"wrote LeRobot dataset: {root}")


if __name__ == "__main__":
    main()
