#!/usr/bin/env python3
"""Jointly calibrate GEM segment hierarchy, position offsets, and rotations."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
import redis


def normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    quaternion = quaternion / norm
    return quaternion if quaternion[0] >= 0.0 else -quaternion


def mean_quaternion(samples: list[np.ndarray]) -> np.ndarray:
    reference = normalize_quaternion(samples[0])
    aligned = []
    for sample in samples:
        quaternion = normalize_quaternion(sample)
        aligned.append(
            -quaternion if np.dot(quaternion, reference) < 0.0 else quaternion
        )
    accumulator = sum(np.outer(quaternion, quaternion) for quaternion in aligned)
    _, eigenvectors = np.linalg.eigh(accumulator)
    return normalize_quaternion(eigenvectors[:, -1])


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion)
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, np.asarray(matrix, dtype=np.float64).reshape(9))
    return normalize_quaternion(quaternion)


def rotation_error_degrees(first: np.ndarray, second: np.ndarray) -> float:
    cosine = np.clip((np.trace(first.T @ second) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def collect_reference(
    client: redis.Redis,
    key: str,
    duration: float,
    sample_hz: float,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    positions: dict[str, list[np.ndarray]] = {}
    quaternions: dict[str, list[np.ndarray]] = {}
    deadline = time.monotonic() + duration
    period = 1.0 / sample_hz
    while time.monotonic() < deadline:
        started = time.monotonic()
        raw = client.get(key)
        if raw:
            try:
                frame = json.loads(raw)
            except json.JSONDecodeError:
                frame = {}
            for name, values in frame.items():
                if len(values) < 7:
                    continue
                value = np.asarray(values[:7], dtype=np.float64)
                if not np.isfinite(value).all():
                    continue
                positions.setdefault(name, []).append(value[:3])
                quaternions.setdefault(name, []).append(value[3:7])
        time.sleep(max(0.0, period - (time.monotonic() - started)))

    if not positions:
        raise RuntimeError(f"no segment poses received from Redis key {key!r}")
    return {
        name: (
            np.mean(np.stack(samples), axis=0),
            mean_quaternion(quaternions[name]),
        )
        for name, samples in positions.items()
        if name in quaternions and quaternions[name]
    }


def robot_neutral_poses(
    xml_path: Path, robot_bodies: set[str]
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    poses = {}
    for body_name in robot_bodies:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise RuntimeError(f"E1 XML is missing body {body_name!r}")
        poses[body_name] = (
            np.asarray(data.xpos[body_id], dtype=np.float64).copy(),
            np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3).copy(),
        )
    return poses


def calibrate_hierarchy(
    adapter: dict,
    human: dict[str, tuple[np.ndarray, np.ndarray]],
    robot: dict[str, tuple[np.ndarray, np.ndarray]],
    human_to_robot: dict[str, str],
) -> tuple[dict, dict[str, np.ndarray]]:
    output = json.loads(json.dumps(adapter))
    root = "Pelvis"
    root_robot = human_to_robot[root]
    human_root_z = float(human[root][0][2])
    robot_root_z = float(robot[root_robot][0][2])
    if abs(human_root_z) < 1e-6:
        raise RuntimeError(
            "human pelvis height is too small for root scale calibration"
        )
    root_correction = robot_root_z / human_root_z
    output["root_translation_scale"] = round(
        float(adapter["root_translation_scale"]) * root_correction, 8
    )

    scaled_positions = {root: human[root][0] * root_correction}
    for parent, child in adapter["hierarchy"]:
        if parent not in human or child not in human:
            raise RuntimeError(f"reference is missing hierarchy edge {parent}->{child}")
        robot_parent = human_to_robot[parent]
        robot_child = human_to_robot[child]
        human_edge = human[child][0] - human[parent][0]
        robot_edge = robot[robot_child][0] - robot[robot_parent][0]
        human_length = float(np.linalg.norm(human_edge))
        robot_length = float(np.linalg.norm(robot_edge))
        if human_length < 1e-6:
            raise RuntimeError(f"degenerate human hierarchy edge {parent}->{child}")
        correction = robot_length / human_length
        edge_name = f"{parent}->{child}"
        output["edge_scales"][edge_name] = round(
            float(adapter["edge_scales"][edge_name]) * correction, 8
        )
        scaled_positions[child] = scaled_positions[parent] + correction * human_edge
        print(
            f"[scale] {edge_name:42s} human={human_length:.5f}m "
            f"E1={robot_length:.5f}m correction={correction:.6f}"
        )
    return output, scaled_positions


def adapter_positions_from_joints(
    adapter: dict, joints: np.ndarray
) -> dict[str, np.ndarray]:
    """Rebuild configured segment origins when alpha is calibrated from debug joints."""
    raw = {}
    for name in adapter["segment_order"]:
        origin = adapter["segments"][name]["origin"]
        proximal = joints[int(origin["proximal_joint"])]
        distal = joints[int(origin["distal_joint"])]
        raw[name] = proximal + float(origin["alpha"]) * (distal - proximal)

    scaled = {"Pelvis": raw["Pelvis"] * float(adapter["root_translation_scale"])}
    for parent, child in adapter["hierarchy"]:
        edge_name = f"{parent}->{child}"
        scaled[child] = scaled[parent] + float(adapter["edge_scales"][edge_name]) * (
            raw[child] - raw[parent]
        )
    return scaled


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--xml", type=Path, default=repo_root / "assets/e1/mjcf/e1_24dof.xml"
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=repo_root / "config/ik_configs/gem_segments_to_e1.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=repo_root / "config/ik_configs/gem_segments_to_e1_calibrated.json",
    )
    parser.add_argument(
        "--adapter-config",
        type=Path,
        default=Path("/home/weili/GENMO/config/gmr/e1_segment_adapter.json"),
    )
    parser.add_argument(
        "--adapter-output",
        type=Path,
        default=Path("/home/weili/GENMO/config/gmr/e1_segment_adapter_calibrated.json"),
    )
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-key", default="gmt_online_frame_e1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    parser.add_argument(
        "--origin-alpha",
        action="append",
        default=[],
        metavar="SEGMENT=ALPHA",
        help="Optional calibrated segment-origin fraction override; repeatable",
    )
    parser.add_argument(
        "--debug-npz",
        type=Path,
        help="Diagnostic NPZ recorded concurrently; required with --origin-alpha",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = json.loads(args.base_config.read_text())
    adapter = json.loads(args.adapter_config.read_text())
    if any(float(value) != 1.0 for value in config["human_scale_table"].values()):
        raise RuntimeError("GEM segment config human_scale_table must be all 1.0")

    table = config["ik_match_table2"]
    human_to_robot = {entry[0]: robot_body for robot_body, entry in table.items()}
    expected = set(adapter["segment_order"])
    if set(human_to_robot) != expected:
        raise RuntimeError("IK mapping and adapter segment names do not match")

    client = redis.Redis(host=args.redis_host, port=args.redis_port, db=args.redis_db)
    raw_key = args.redis_key + "_raw_bones"
    print(
        "Hold E1 neutral reference: upright, feet parallel, arms down, elbows "
        f"straight. Collecting {args.duration:.1f}s from {raw_key} ..."
    )
    human = collect_reference(client, raw_key, args.duration, args.sample_hz)
    missing = expected - set(human)
    if missing:
        raise RuntimeError(f"reference is missing segments: {sorted(missing)}")

    if args.origin_alpha and args.debug_npz is None:
        raise ValueError("--origin-alpha requires --debug-npz with raw SMPL joints")
    for override in args.origin_alpha:
        if "=" not in override:
            raise ValueError(
                f"invalid --origin-alpha {override!r}; expected SEGMENT=ALPHA"
            )
        segment_name, value_text = override.split("=", 1)
        if segment_name not in adapter["segments"]:
            raise ValueError(f"unknown segment in --origin-alpha: {segment_name!r}")
        alpha = float(value_text)
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("origin alpha must be in [0, 1]")
        adapter["segments"][segment_name]["origin"]["alpha"] = alpha
        print(f"[origin] {segment_name} alpha={alpha:.6f}")
    if args.debug_npz is not None:
        diagnostic = np.load(args.debug_npz)
        raw_joints = np.asarray(diagnostic["raw_joints"], dtype=np.float64)
        if raw_joints.ndim != 3 or raw_joints.shape[1:] != (22, 3):
            raise ValueError("debug NPZ raw_joints must have shape (frames,22,3)")
        recalculated = adapter_positions_from_joints(
            adapter, np.mean(raw_joints, axis=0)
        )
        human = {
            name: (recalculated[name], human[name][1])
            for name in adapter["segment_order"]
        }
        print(f"[origin] recalculated segment positions from {args.debug_npz}")

    robot = robot_neutral_poses(args.xml, set(human_to_robot.values()))
    adapter_calibrated, calibrated_positions = calibrate_hierarchy(
        adapter, human, robot, human_to_robot
    )

    offsets = {}
    position_residuals = {}
    rotation_residuals = {}
    for human_name in adapter["segment_order"]:
        robot_name = human_to_robot[human_name]
        human_rotation = quaternion_to_matrix(human[human_name][1])
        robot_position, robot_rotation = robot[robot_name]
        human_position = calibrated_positions[human_name]

        rotation_offset = human_rotation.T @ robot_rotation
        position_offset = robot_rotation.T @ (robot_position - human_position)
        offsets[human_name] = (
            position_offset,
            matrix_to_quaternion(rotation_offset),
        )

        target_rotation = human_rotation @ rotation_offset
        target_position = human_position + target_rotation @ position_offset
        position_residuals[human_name] = float(
            np.linalg.norm(target_position - robot_position)
        )
        rotation_residuals[human_name] = rotation_error_degrees(
            target_rotation, robot_rotation
        )
        print(
            f"[offset] {robot_name:28s} <- {human_name:20s} "
            f"pos_res={position_residuals[human_name]:.3e}m "
            f"rot_res={rotation_residuals[human_name]:.3e}deg"
        )

    for table_name in ("ik_match_table1", "ik_match_table2"):
        for _, entry in config[table_name].items():
            position_offset, rotation_offset = offsets[entry[0]]
            entry[3] = [round(float(value), 8) for value in position_offset]
            entry[4] = [round(float(value), 8) for value in rotation_offset]

    for left_name in sorted(name for name in expected if name.startswith("Left_")):
        right_name = left_name.replace("Left_", "Right_", 1)
        position_delta = abs(
            position_residuals[left_name] - position_residuals[right_name]
        )
        rotation_delta = abs(
            rotation_residuals[left_name] - rotation_residuals[right_name]
        )
        print(
            f"[symmetry] {left_name[5:]:14s} residual_delta="
            f"{position_delta:.3e}m/{rotation_delta:.3e}deg"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.adapter_output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2) + "\n")
    args.adapter_output.write_text(json.dumps(adapter_calibrated, indent=2) + "\n")
    print(f"Wrote IK config: {args.output}")
    print(f"Wrote adapter config: {args.adapter_output}")
    print(
        f"Maximum reference residual: {max(position_residuals.values()):.3e}m, "
        f"{max(rotation_residuals.values()):.3e}deg"
    )


if __name__ == "__main__":
    main()
