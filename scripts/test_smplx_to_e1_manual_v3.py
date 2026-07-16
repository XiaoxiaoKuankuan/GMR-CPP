#!/usr/bin/env python3
"""Read-only fixed-pose SMP1 -> E1 manual-v3 end-to-end diagnostic.

The script deliberately contains no calibration, search, or configuration-write
path.  Pose rules are reported as PASS/WARN; a WARN never changes the config.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import redis


ROOT = Path(__file__).resolve().parents[1]
TARGET_NAMES = (
    "pelvis",
    "spine3",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_foot",
    "right_foot",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
)
HEADER = struct.Struct("<4sHHIQ")
PAYLOAD = struct.Struct("<" + "f" * (len(TARGET_NAMES) * 7))
E1_JOINT_IDS_MAP = (
    0, 6, 12, 1, 7, 13, 2, 8, 14, 19, 3, 9,
    15, 20, 4, 10, 16, 21, 5, 11, 17, 22, 18, 23,
)
KEY_JOINTS = (
    "l_arm_shoulder_pitch_joint",
    "l_arm_shoulder_roll_joint",
    "l_arm_shoulder_yaw_joint",
    "l_arm_elbow_pitch_joint",
    "l_arm_elbow_yaw_joint",
    "r_arm_shoulder_pitch_joint",
    "r_arm_shoulder_roll_joint",
    "r_arm_shoulder_yaw_joint",
    "r_arm_elbow_pitch_joint",
    "r_arm_elbow_yaw_joint",
    "l_leg_knee_joint",
    "r_leg_knee_joint",
)
KEY_BODIES = (
    "base_link",
    "l_leg_ankle_roll_link",
    "r_leg_ankle_roll_link",
    "l_arm_shoulder_roll_link",
    "r_arm_shoulder_roll_link",
    "l_arm_elbow_yaw_link",
    "r_arm_elbow_yaw_link",
)


@dataclass(frozen=True)
class Target:
    position: np.ndarray
    quaternion: np.ndarray


@dataclass
class Result:
    qpos: np.ndarray
    joints: dict[str, float]
    body_z: dict[str, float]
    touched_limits: list[str]
    position_error_mean: float
    position_error_max: float
    rotation_error_mean: float
    rotation_error_max: float


def rotation_y(degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray(
        ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine)),
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return a normalized wxyz quaternion for a proper 3x3 matrix."""
    matrix = np.asarray(matrix, dtype=np.float64)
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, matrix.reshape(9))
    quat /= np.linalg.norm(quat)
    return quat


def multiply_quaternions(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    value = np.asarray(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        )
    )
    return value / np.linalg.norm(value)


def quaternion_error(left: np.ndarray, right: np.ndarray) -> float:
    dot = float(abs(np.dot(left / np.linalg.norm(left), right / np.linalg.norm(right))))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def base_positions() -> dict[str, tuple[float, float, float]]:
    # Deterministic normalized Z-up joint centers, matching the SMP1 target semantics.
    return {
        "pelvis": (0.00, 0.00, 1.00),
        "spine3": (0.00, 0.00, 1.35),
        "left_hip": (0.00, 0.12, 0.95),
        "right_hip": (0.00, -0.12, 0.95),
        "left_knee": (0.00, 0.12, 0.55),
        "right_knee": (0.00, -0.12, 0.55),
        "left_foot": (0.20, 0.12, 0.05),
        "right_foot": (0.20, -0.12, 0.05),
        "left_shoulder": (0.00, 0.25, 1.40),
        "right_shoulder": (0.00, -0.25, 1.40),
        "left_elbow": (0.00, 0.50, 1.20),
        "right_elbow": (0.00, -0.50, 1.20),
        "left_wrist": (0.00, 0.70, 1.00),
        "right_wrist": (0.00, -0.70, 1.00),
    }


def pose_from_positions(
    positions: dict[str, tuple[float, float, float]],
    local_rotation_degrees: dict[str, float] | None = None,
) -> dict[str, Target]:
    # The current GENMO reference applies AY->Z-up to global FK frames on the left.
    axis_convert = np.asarray(
        ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
        dtype=np.float64,
    )
    local_rotation_degrees = local_rotation_degrees or {}
    return {
        name: Target(
            np.asarray(positions[name], dtype=np.float64),
            matrix_to_quaternion(
                axis_convert @ rotation_y(local_rotation_degrees.get(name, 0.0))
            ),
        )
        for name in TARGET_NAMES
    }


def make_poses() -> dict[str, dict[str, Target]]:
    poses: dict[str, dict[str, Target]] = {}

    poses["stand"] = pose_from_positions(base_positions())

    t_pose = base_positions()
    t_pose.update(
        left_elbow=(0.00, 0.60, 1.40), right_elbow=(0.00, -0.60, 1.40),
        left_wrist=(0.00, 0.90, 1.40), right_wrist=(0.00, -0.90, 1.40),
    )
    poses["T-pose"] = pose_from_positions(t_pose)

    arms_down = base_positions()
    arms_down.update(
        left_elbow=(0.00, 0.27, 1.06), right_elbow=(0.00, -0.27, 1.06),
        left_wrist=(0.00, 0.29, 0.74), right_wrist=(0.00, -0.29, 0.74),
    )
    poses["arms-down"] = pose_from_positions(arms_down)

    left_up = base_positions()
    left_up.update(
        left_elbow=(0.00, 0.25, 1.72), left_wrist=(0.00, 0.25, 2.02)
    )
    poses["left-arm-up"] = pose_from_positions(
        left_up, {"left_shoulder": 90.0, "left_elbow": 90.0, "left_wrist": 90.0}
    )

    right_up = base_positions()
    right_up.update(
        right_elbow=(0.00, -0.25, 1.72), right_wrist=(0.00, -0.25, 2.02)
    )
    poses["right-arm-up"] = pose_from_positions(
        right_up,
        {"right_shoulder": -90.0, "right_elbow": -90.0, "right_wrist": -90.0},
    )

    left_flex = dict(t_pose)
    diagonal = 0.30 / math.sqrt(2.0)
    left_flex["left_wrist"] = (0.00, 0.60 + diagonal, 1.40 - diagonal)
    poses["left-elbow-flex-45"] = pose_from_positions(
        left_flex, {"left_elbow": 45.0, "left_wrist": 45.0}
    )

    right_flex = dict(t_pose)
    right_flex["right_wrist"] = (0.00, -0.60 - diagonal, 1.40 - diagonal)
    poses["right-elbow-flex-45"] = pose_from_positions(
        right_flex, {"right_elbow": -45.0, "right_wrist": -45.0}
    )

    squat = base_positions()
    squat.update(
        pelvis=(0.00, 0.00, 0.70), spine3=(0.00, 0.00, 1.05),
        left_hip=(0.00, 0.12, 0.68), right_hip=(0.00, -0.12, 0.68),
        left_knee=(0.22, 0.12, 0.39), right_knee=(0.22, -0.12, 0.39),
        left_foot=(0.25, 0.12, 0.05), right_foot=(0.25, -0.12, 0.05),
        left_shoulder=(0.00, 0.25, 1.10), right_shoulder=(0.00, -0.25, 1.10),
        left_elbow=(0.00, 0.27, 0.78), right_elbow=(0.00, -0.27, 0.78),
        left_wrist=(0.00, 0.29, 0.48), right_wrist=(0.00, -0.29, 0.48),
    )
    poses["squat"] = pose_from_positions(
        squat,
        {
            "left_hip": 35.0, "right_hip": 35.0,
            "left_knee": -70.0, "right_knee": -70.0,
            "left_foot": -5.0, "right_foot": -5.0,
        },
    )
    return poses


class PoseStreamer:
    def __init__(self, port: int, pose: dict[str, Target], hz: float = 100.0):
        self.port = port
        self.hz = hz
        self._pose = pose
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sequence = 0
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def set_pose(self, pose: dict[str, Target]) -> None:
        with self._lock:
            self._pose = pose

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._socket.close()

    def _packet(self, pose: dict[str, Target]) -> bytes:
        values: list[float] = []
        for name in TARGET_NAMES:
            target = pose[name]
            values.extend((*target.position.tolist(), *target.quaternion.tolist()))
        packet = HEADER.pack(
            b"SMP1", 1, len(TARGET_NAMES), self._sequence & 0xFFFFFFFF,
            time.monotonic_ns(),
        ) + PAYLOAD.pack(*values)
        self._sequence += 1
        return packet

    def _run(self) -> None:
        period = 1.0 / self.hz
        next_tick = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                pose = self._pose
            self._socket.sendto(self._packet(pose), ("127.0.0.1", self.port))
            next_tick += period
            self._stop.wait(max(0.0, next_tick - time.monotonic()))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def validate_fixed_config(config_path: Path) -> dict:
    config = load_json(config_path)
    g1 = load_json(ROOT / "config/ik_configs/smplx_to_g1.json")
    xrobot = load_json(ROOT / "config/ik_configs/xrobot_to_e1.json")

    assert config["human_root_name"] == g1["human_root_name"]
    assert config["human_scale_table"] == g1["human_scale_table"]
    for table_name in ("ik_match_table1", "ik_match_table2"):
        assert tuple(config[table_name]) == tuple(xrobot[table_name])
        for robot_body, entry in config[table_name].items():
            assert entry[3] == [0, 0, 0]
            assert np.allclose(entry[4], xrobot[table_name][robot_body][4])

    table1 = config["ik_match_table1"]
    table2 = config["ik_match_table2"]
    for side in ("l", "r"):
        assert table1[f"{side}_leg_ankle_roll_link"][2] == 8
        assert table2[f"{side}_leg_ankle_roll_link"][2] == 8
        assert table1[f"{side}_arm_elbow_yaw_link"][2] == 1
        assert table2[f"{side}_arm_elbow_yaw_link"][2] == 0
        assert table2[f"{side}_leg_knee_link"][1] == 20
    assert table2["base_link"][1] == 30
    print("[PASS] fixed config invariants: G1 names/scales, XRobot E1 mapping/offsets, manual weights")
    print("[PASS] config is read-only: this test has no config-write/search/calibration path")
    return config


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = int(sock.getsockname()[1])
    sock.close()
    return port


def wait_for_pose(
    client: redis.Redis,
    raw_key: str,
    expected: dict[str, Target],
    process: subprocess.Popen[str],
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"smplx_e1_server exited early with {process.returncode}")
        raw = client.get(raw_key)
        if raw:
            decoded = json.loads(raw)
            if all(
                name in decoded
                and np.linalg.norm(np.asarray(decoded[name][:3]) - expected[name].position)
                < 2e-5
                for name in TARGET_NAMES
            ):
                return
        time.sleep(0.02)
    raise TimeoutError(f"no current pose received on Redis key {raw_key!r}")


def payload_to_qpos(payload: bytes, model: mujoco.MjModel) -> np.ndarray:
    values = np.frombuffer(payload, dtype="<f4")
    if values.size != 38:
        raise ValueError(f"E1 Redis payload must contain 38 floats, got {values.size}")
    qpos = np.asarray(model.qpos0, dtype=np.float64).copy()
    qpos[:3] = values[1:4]
    qpos[3:7] = values[4:8]
    for controller_index, qpos_index in enumerate(E1_JOINT_IDS_MAP):
        qpos[7 + qpos_index] = values[14 + controller_index]
    return qpos


def joint_value(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return float(data.qpos[model.jnt_qposadr[joint_id]])


def body_z(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> float:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return float(data.xpos[body_id, 2])


def scaled_targets(config: dict, pose: dict[str, Target]) -> dict[str, Target]:
    root_name = config["human_root_name"]
    root_position = pose[root_name].position
    root_scale = float(config["human_scale_table"][root_name])
    scaled_root = root_position * root_scale
    output: dict[str, Target] = {}
    for name, target in pose.items():
        scale = float(config["human_scale_table"][name])
        position = (
            scaled_root if name == root_name
            else (target.position - root_position) * scale + scaled_root
        )
        output[name] = Target(position, target.quaternion)
    return output


def target_errors(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: dict,
    pose: dict[str, Target],
) -> tuple[float, float, float, float]:
    scaled = scaled_targets(config, pose)
    position_errors: list[float] = []
    rotation_errors: list[float] = []
    for robot_body, entry in config["ik_match_table2"].items():
        human_name, position_weight, rotation_weight, position_offset, rotation_offset = entry
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
        target = scaled[human_name]
        target_quaternion = multiply_quaternions(
            target.quaternion, np.asarray(rotation_offset, dtype=np.float64)
        )
        position = target.position.copy()
        if np.linalg.norm(position_offset) > 0.0:
            rotation_matrix = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(rotation_matrix, target_quaternion)
            position += rotation_matrix.reshape(3, 3) @ np.asarray(position_offset)
        if position_weight:
            position_errors.append(float(np.linalg.norm(data.xpos[body_id] - position)))
        if rotation_weight:
            rotation_errors.append(quaternion_error(data.xquat[body_id], target_quaternion))
    return (
        float(np.mean(position_errors)), float(np.max(position_errors)),
        float(np.mean(rotation_errors)), float(np.max(rotation_errors)),
    )


def inspect_result(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: dict,
    pose: dict[str, Target],
    payload: bytes,
) -> Result:
    qpos = payload_to_qpos(payload, model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    joints = {name: joint_value(model, data, name) for name in KEY_JOINTS}
    heights = {name: body_z(model, data, name) for name in KEY_BODIES}
    touched: list[str] = []
    for joint_id in range(model.njnt):
        if not model.jnt_limited[joint_id]:
            continue
        qpos_address = model.jnt_qposadr[joint_id]
        value = float(data.qpos[qpos_address])
        low, high = model.jnt_range[joint_id]
        if min(abs(value - low), abs(high - value)) <= 1e-3:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            touched.append(f"{name}={value:.3f}[{low:.3f},{high:.3f}]")
    pos_mean, pos_max, rot_mean, rot_max = target_errors(model, data, config, pose)
    return Result(qpos, joints, heights, touched, pos_mean, pos_max, rot_mean, rot_max)


def print_result(name: str, result: Result) -> None:
    print(f"\n=== {name} ===")
    print("E1 qpos:", np.array2string(result.qpos, precision=4, suppress_small=True))
    print(
        "key joints:",
        " ".join(f"{joint.replace('_joint', '')}={value:+.3f}" for joint, value in result.joints.items()),
    )
    print("body z:", " ".join(f"{body}={value:.3f}" for body, value in result.body_z.items()))
    print("joint limits:", ", ".join(result.touched_limits) if result.touched_limits else "none")
    print(
        "target errors: "
        f"position mean/max={result.position_error_mean:.4f}/{result.position_error_max:.4f} m; "
        f"rotation mean/max={math.degrees(result.rotation_error_mean):.2f}/"
        f"{math.degrees(result.rotation_error_max):.2f} deg"
    )


def report_rule(label: str, passed: bool, detail: str) -> None:
    print(f"[{'PASS' if passed else 'WARN'}] {label}: {detail}")


def report_pose_rules(results: dict[str, Result]) -> None:
    print("\n=== fixed-pose rule report (read-only) ===")
    t_pose = results["T-pose"]
    left_roll = t_pose.joints["l_arm_shoulder_roll_joint"]
    right_roll = t_pose.joints["r_arm_shoulder_roll_joint"]
    left_elbow = t_pose.joints["l_arm_elbow_pitch_joint"]
    right_elbow = t_pose.joints["r_arm_elbow_pitch_joint"]
    symmetric = left_roll * right_roll < 0.0 and abs(abs(left_roll) - abs(right_roll)) < 0.35
    report_rule(
        "T-pose shoulder opposition/symmetry", symmetric,
        f"roll L/R={left_roll:+.3f}/{right_roll:+.3f}",
    )
    report_rule(
        "T-pose elbows near zero/no large reverse",
        max(abs(left_elbow), abs(right_elbow)) < 0.45,
        f"elbow_pitch L/R={left_elbow:+.3f}/{right_elbow:+.3f}",
    )

    arms_down = results["arms-down"]
    down_elbows = (
        abs(arms_down.joints["l_arm_elbow_pitch_joint"]) < 0.45
        and abs(arms_down.joints["r_arm_elbow_pitch_joint"]) < 0.45
    )
    links_down = (
        arms_down.body_z["l_arm_elbow_yaw_link"] < arms_down.body_z["l_arm_shoulder_roll_link"]
        and arms_down.body_z["r_arm_elbow_yaw_link"] < arms_down.body_z["r_arm_shoulder_roll_link"]
    )
    report_rule(
        "arms-down elbows/links", down_elbows and links_down,
        f"elbow_pitch L/R={arms_down.joints['l_arm_elbow_pitch_joint']:+.3f}/"
        f"{arms_down.joints['r_arm_elbow_pitch_joint']:+.3f}, links_down={links_down}",
    )

    left_flex = results["left-elbow-flex-45"].joints["l_arm_elbow_pitch_joint"]
    right_flex = results["right-elbow-flex-45"].joints["r_arm_elbow_pitch_joint"]
    report_rule(
        "left elbow flexes in negative E1 direction", left_flex < left_elbow - 0.05,
        f"T-pose={left_elbow:+.3f}, flex45={left_flex:+.3f}",
    )
    report_rule(
        "right elbow flexes in negative E1 direction", right_flex < right_elbow - 0.05,
        f"T-pose={right_elbow:+.3f}, flex45={right_flex:+.3f}",
    )

    stand = results["stand"]
    squat = results["squat"]
    base_drop = stand.body_z["base_link"] - squat.body_z["base_link"]
    left_knee_change = squat.joints["l_leg_knee_joint"] - stand.joints["l_leg_knee_joint"]
    right_knee_change = squat.joints["r_leg_knee_joint"] - stand.joints["r_leg_knee_joint"]
    left_foot_rise = squat.body_z["l_leg_ankle_roll_link"] - stand.body_z["l_leg_ankle_roll_link"]
    right_foot_rise = squat.body_z["r_leg_ankle_roll_link"] - stand.body_z["r_leg_ankle_roll_link"]
    report_rule("squat base descends", base_drop > 0.10, f"drop={base_drop:.3f} m")
    report_rule(
        "squat knees flex more", min(left_knee_change, right_knee_change) > 0.10,
        f"delta L/R={left_knee_change:+.3f}/{right_knee_change:+.3f} rad",
    )
    report_rule(
        "squat feet do not visibly lift", max(left_foot_rise, right_foot_rise) < 0.08,
        f"foot z delta L/R={left_foot_rise:+.3f}/{right_foot_rise:+.3f} m",
    )
    print("[INFO] PASS/WARN above is diagnostic only; no parameter was changed.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", type=Path, default=ROOT / "build/smplx_e1_server")
    parser.add_argument("--xml", type=Path, default=ROOT / "assets/e1/mjcf/e1_24dof.xml")
    parser.add_argument(
        "--config", type=Path,
        default=ROOT / "config/ik_configs/smplx_to_e1_manual_v3.json",
    )
    parser.add_argument("--port", type=int, default=0, help="UDP port; 0 selects a free port")
    parser.add_argument("--settle", type=float, default=0.8, help="seconds per pose")
    parser.add_argument("--redis-key", default="smplx_manual_v3_fixed_pose_test")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for path in (args.server, args.xml, args.config):
        if not path.is_file():
            raise FileNotFoundError(path)
    config = validate_fixed_config(args.config)
    poses = make_poses()
    model = mujoco.MjModel.from_xml_path(str(args.xml))
    data = mujoco.MjData(model)
    client = redis.Redis(host="127.0.0.1", port=6379, db=0)
    if not client.ping():
        raise RuntimeError("Redis 127.0.0.1:6379 did not answer PING")

    port = args.port or free_udp_port()
    keys = (args.redis_key, args.redis_key + ":stream", args.redis_key + "_raw_bones")
    client.delete(*keys)
    environment = os.environ.copy()
    library_path = str(ROOT / "third_party/mujoco/lib")
    environment["LD_LIBRARY_PATH"] = library_path + ":" + environment.get("LD_LIBRARY_PATH", "")
    command = [
        str(args.server), "--bind", "127.0.0.1", "--port", str(port),
        "--xml", str(args.xml), "--ik-config", str(args.config),
        "--redis-key", args.redis_key, "--ttl-ms", "2000", "--stale-ms", "1000",
        "--no-offset-to-ground", "--always",
    ]
    print("[test]", " ".join(command))
    process = subprocess.Popen(
        command, cwd=ROOT, env=environment, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True,
    )
    streamer = PoseStreamer(port, poses["stand"])
    streamer.start()
    results: dict[str, Result] = {}
    try:
        for index, (name, pose) in enumerate(poses.items()):
            streamer.set_pose(pose)
            wait_for_pose(
                client, args.redis_key + "_raw_bones", pose, process,
                timeout=20.0 if index == 0 else 5.0,
            )
            time.sleep(args.settle)
            payload = client.get(args.redis_key)
            if payload is None:
                raise RuntimeError(f"no E1 frame on Redis key {args.redis_key!r}")
            result = inspect_result(model, data, config, pose, payload)
            results[name] = result
            print_result(name, result)
        report_pose_rules(results)
    finally:
        streamer.close()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5.0)
        print("\n=== smplx_e1_server tail ===")
        print("\n".join(output.splitlines()[-30:]))
        client.delete(*keys)
    if process.returncode not in (0, -15):
        raise RuntimeError(f"smplx_e1_server returned {process.returncode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
