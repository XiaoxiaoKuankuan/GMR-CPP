#!/usr/bin/env python3
"""Calibrate GEM position scales from an E1 neutral MuJoCo FK pose.

Robot segment lengths are measured from the configured E1 XML at runtime;
there are deliberately no copied or hard-coded robot length constants here.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
import redis

ROBOT_BODY_PAIRS = {
    "thigh": (
        ("l_leg_hip_pitch_link", "l_leg_knee_link"),
        ("r_leg_hip_pitch_link", "r_leg_knee_link"),
    ),
    "shank": (
        ("l_leg_knee_link", "l_leg_ankle_roll_link"),
        ("r_leg_knee_link", "r_leg_ankle_roll_link"),
    ),
    "trunk": (("base_link", "waist_roll_link"),),
    "upper_arm": (
        ("l_arm_shoulder_roll_link", "l_arm_elbow_pitch_link"),
        ("r_arm_shoulder_roll_link", "r_arm_elbow_pitch_link"),
    ),
    "forearm": (
        ("l_arm_elbow_pitch_link", "l_arm_elbow_yaw_link"),
        ("r_arm_elbow_pitch_link", "r_arm_elbow_yaw_link"),
    ),
}

HUMAN_BODY_PAIRS = {
    "thigh": (
        ("Left_UpperLeg", "Left_LowerLeg"),
        ("Right_UpperLeg", "Right_LowerLeg"),
    ),
    "shank": (
        ("Left_LowerLeg", "Left_Foot"),
        ("Right_LowerLeg", "Right_Foot"),
    ),
    "trunk": (("Pelvis", "Chest"),),
    "upper_arm": (
        ("Left_UpperArm", "Left_Forearm"),
        ("Right_UpperArm", "Right_Forearm"),
    ),
    "forearm": (
        ("Left_Forearm", "Left_Hand"),
        ("Right_Forearm", "Right_Hand"),
    ),
}


def neutral_robot_lengths(xml_path: Path) -> dict[str, float]:
    """Measure bilateral E1 segments after neutral-pose MuJoCo FK."""
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    lengths: dict[str, float] = {}
    for segment, pairs in ROBOT_BODY_PAIRS.items():
        pair_lengths = []
        for first, second in pairs:
            ids = []
            for body_name in (first, second):
                body_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, body_name
                )
                if body_id < 0:
                    raise RuntimeError(
                        f"E1 XML is missing required body {body_name!r}"
                    )
                ids.append(body_id)
            pair_lengths.append(
                float(np.linalg.norm(data.xpos[ids[1]] - data.xpos[ids[0]]))
            )
        lengths[segment] = float(np.mean(pair_lengths))
    return lengths


def collect(
    client: redis.Redis, key: str, duration: float, sample_hz: float
) -> list[dict[str, list[float]]]:
    frames = []
    deadline = time.monotonic() + duration
    period = 1.0 / sample_hz
    while time.monotonic() < deadline:
        started = time.monotonic()
        raw = client.get(key)
        if raw:
            try:
                frames.append(json.loads(raw))
            except json.JSONDecodeError:
                pass
        time.sleep(max(0.0, period - (time.monotonic() - started)))
    return frames


def median_human_lengths(
    frames: list[dict[str, list[float]]],
) -> dict[str, float]:
    values: dict[str, list[float]] = {name: [] for name in HUMAN_BODY_PAIRS}
    for frame in frames:
        for segment, pairs in HUMAN_BODY_PAIRS.items():
            for first, second in pairs:
                if first not in frame or second not in frame:
                    continue
                p_first = np.asarray(frame[first][:3], dtype=np.float64)
                p_second = np.asarray(frame[second][:3], dtype=np.float64)
                distance = float(np.linalg.norm(p_first - p_second))
                if 0.03 < distance < 2.0:
                    values[segment].append(distance)

    result = {}
    for segment, samples in values.items():
        if not samples:
            raise RuntimeError(f"no valid GEM samples for {segment}")
        result[segment] = float(np.median(samples))
    return result


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--xml", default=repo_root / "assets/e1/mjcf/e1_24dof.xml", type=Path
    )
    parser.add_argument(
        "--base-config",
        default=repo_root / "config/ik_configs/gem_to_e1_position.json",
        type=Path,
    )
    parser.add_argument(
        "--output",
        default=repo_root / "config/ik_configs/calibrated/gem_to_e1_scaled.json",
        type=Path,
    )
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-key", default="gmt_online_frame_e1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    args = parser.parse_args()

    robot = neutral_robot_lengths(args.xml)
    client = redis.Redis(
        host=args.redis_host, port=args.redis_port, db=args.redis_db
    )
    raw_key = args.redis_key + "_raw_bones"
    print(f"Hold T-pose; collecting {args.duration:.1f}s from {raw_key} ...")
    frames = collect(client, raw_key, args.duration, args.sample_hz)
    if not frames:
        raise RuntimeError(f"no frames received from {raw_key}")

    human = median_human_lengths(frames)
    scale = {segment: robot[segment] / human[segment] for segment in human}
    root_scale = 0.5 * (scale["thigh"] + scale["shank"])

    config = json.loads(args.base_config.read_text())
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

    for segment in human:
        print(
            f"{segment:10s}: human={human[segment]:.4f}m "
            f"E1={robot[segment]:.4f}m scale={scale[segment]:.4f}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2) + "\n")
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
