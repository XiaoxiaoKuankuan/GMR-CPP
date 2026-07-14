#!/usr/bin/env python3
"""Calibrate GEM-to-G1 position scales from the Redis raw-bone stream.

The G1 segment lengths are the same values used by GMR-CPP's existing
auto_calibrate.py.  This script only changes human_scale_table.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import redis

ROBOT_LENGTHS = {
    "thigh": 0.3061,
    "shank": 0.3521,
    "trunk": 0.0442,
    "upper_arm": 0.0821,
    "forearm": 0.1843,
}

SEGMENT_PAIRS = {
    "thigh": ("Left_UpperLeg", "Left_LowerLeg"),
    "shank": ("Left_LowerLeg", "Left_Foot"),
    "trunk": ("Pelvis", "Chest"),
    "upper_arm": ("Left_UpperArm", "Left_Forearm"),
    "forearm": ("Left_Forearm", "Left_Hand"),
}


def collect(
    client: redis.Redis, key: str, duration: float, sample_hz: float
) -> list[dict[str, list[float]]]:
    frames = []
    deadline = time.monotonic() + duration
    period = 1.0 / sample_hz
    while time.monotonic() < deadline:
        t0 = time.monotonic()
        raw = client.get(key)
        if raw:
            try:
                frames.append(json.loads(raw))
            except json.JSONDecodeError:
                pass
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
    return frames


def median_lengths(frames: list[dict[str, list[float]]]) -> dict[str, float]:
    values: dict[str, list[float]] = {name: [] for name in SEGMENT_PAIRS}
    for frame in frames:
        for segment, (a, b) in SEGMENT_PAIRS.items():
            if a not in frame or b not in frame:
                continue
            pa = np.asarray(frame[a][:3], dtype=np.float64)
            pb = np.asarray(frame[b][:3], dtype=np.float64)
            d = float(np.linalg.norm(pa - pb))
            if 0.03 < d < 2.0:
                values[segment].append(d)

    result = {}
    for segment, samples in values.items():
        if not samples:
            raise RuntimeError(f"no valid samples for {segment}")
        result[segment] = float(np.median(samples))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-key", default="mmocap_motion_frame_g1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    args = parser.parse_args()

    client = redis.Redis(
        host=args.redis_host, port=args.redis_port, db=args.redis_db
    )
    raw_key = args.redis_key + "_raw_bones"
    print(f"Hold T-pose; collecting {args.duration:.1f}s from {raw_key} ...")
    frames = collect(client, raw_key, args.duration, args.sample_hz)
    if not frames:
        raise RuntimeError(f"no frames received from {raw_key}")

    human = median_lengths(frames)
    scale = {k: ROBOT_LENGTHS[k] / human[k] for k in human}
    root_scale = 0.5 * (scale["thigh"] + scale["shank"])

    config = json.loads(Path(args.base_config).read_text())
    config["human_scale_table"] = {
        "Pelvis": round(root_scale, 4),
        "Chest": round(scale["trunk"], 4),
        "Left_UpperLeg": round(scale["thigh"], 4),
        "Right_UpperLeg": round(scale["thigh"], 4),
        "Left_LowerLeg": round(scale["shank"], 4),
        "Right_LowerLeg": round(scale["shank"], 4),
        "Left_Foot": round(scale["shank"], 4),
        "Right_Foot": round(scale["shank"], 4),
        "Left_UpperArm": round(scale["upper_arm"], 4),
        "Right_UpperArm": round(scale["upper_arm"], 4),
        "Left_Forearm": round(scale["forearm"], 4),
        "Right_Forearm": round(scale["forearm"], 4),
        "Left_Hand": round(scale["forearm"], 4),
        "Right_Hand": round(scale["forearm"], 4),
    }

    for name in human:
        print(
            f"{name:10s}: human={human[name]:.4f}m "
            f"robot={ROBOT_LENGTHS[name]:.4f}m scale={scale[name]:.4f}"
        )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + "\n")
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
