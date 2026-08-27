#!/usr/bin/env python3
"""Validate fixed SMPL-X -> BUMI3 JSON and model without modifying either."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

try:
    import mujoco
except ModuleNotFoundError as error:  # pragma: no cover - environment guidance
    raise SystemExit(
        "mujoco Python package is required; use /home/weili/GENMO/.venv/bin/python"
    ) from error


ROOT = Path(__file__).resolve().parents[1]
SMP1_TARGETS = {
    "pelvis", "spine3", "left_hip", "right_hip", "left_knee",
    "right_knee", "left_foot", "right_foot", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist",
    "right_wrist",
}
GMT_JOINT_ORDER = [
    "l_leg_pitch_joint", "r_leg_pitch_joint", "waist_yaw_joint",
    "l_leg_roll_joint", "r_leg_roll_joint",
    "l_arm_pitch_joint", "r_arm_pitch_joint",
    "l_leg_yaw_joint", "r_leg_yaw_joint",
    "l_arm_roll_joint", "r_arm_roll_joint",
    "l_knee_pitch_joint", "r_knee_pitch_joint",
    "l_arm_yaw_joint", "r_arm_yaw_joint",
    "l_ankle_pitch_joint", "r_ankle_pitch_joint",
    "l_elbow_pitch_joint", "r_elbow_pitch_joint",
    "l_ankle_roll_joint", "r_ankle_roll_joint",
]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--expected-sha256")
    parser.add_argument(
        "--robot-preset", type=Path,
        default=ROOT / "config/robot_presets/bumi3.json",
    )
    args = parser.parse_args()
    xml_path = args.xml.expanduser().resolve(strict=True)
    config_path = args.config.expanduser().resolve(strict=True)
    robot_preset_path = args.robot_preset.expanduser().resolve(strict=True)

    runtime_hash = sha256(config_path)
    if args.expected_sha256:
        require(
            runtime_hash == args.expected_sha256,
            f"runtime config SHA mismatch: {runtime_hash}",
        )
    config = json.loads(config_path.read_text())
    require(config["robot_root_name"] == "base_link", "robot root must be base_link")
    require(config["human_root_name"] == "pelvis", "human root must be pelvis")
    require(config["use_ik_match_table1"], "IK table1 must be enabled")
    require(config["use_ik_match_table2"], "IK table2 must be enabled")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    body_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or ""
        for index in range(model.nbody)
    ]
    duplicates = sorted(
        name for name, count in Counter(body_names).items() if name and count > 1
    )
    require(not duplicates, f"duplicate body names: {duplicates}")

    table1 = config["ik_match_table1"]
    table2 = config["ik_match_table2"]
    require(set(table1) == set(table2), "table1/table2 robot body sets differ")
    require(len(table1) == 12, f"expected 12 configured targets, got {len(table1)}")
    missing = sorted(set(table1) - set(body_names))
    require(not missing, f"configured robot bodies missing from XML: {missing}")

    used_humans: set[str] = set()
    for table_name, table in (("table1", table1), ("table2", table2)):
        for robot_body, entry in table.items():
            require(len(entry) == 5, f"{table_name}.{robot_body} entry length != 5")
            human = entry[0]
            used_humans.add(human)
            require(human in SMP1_TARGETS, f"unsupported SMP1 target: {human}")
            require(
                all(math.isfinite(float(value)) for value in entry[3]),
                f"non-finite position offset: {table_name}.{robot_body}",
            )
            quaternion = [float(value) for value in entry[4]]
            require(all(math.isfinite(value) for value in quaternion), "non-finite quaternion")
            norm = math.sqrt(sum(value * value for value in quaternion))
            require(
                abs(norm - 1.0) <= 1e-6,
                f"quaternion norm {norm} at {table_name}.{robot_body}",
            )
    require("left_wrist" not in used_humans, "left_wrist must remain unused")
    require("right_wrist" not in used_humans, "right_wrist must remain unused")
    require(
        all(math.isfinite(float(value)) and float(value) > 0.0
            for value in config["human_scale_table"].values()),
        "human scales must be finite and positive",
    )
    foot_contact = config["foot_contact"]
    require(foot_contact["enabled"], "foot-contact profile must be enabled")
    require(foot_contact["allow_flight"], "foot-contact detector must allow flight")
    require(
        foot_contact["source"]["left"]["body"] == "left_foot" and
        foot_contact["source"]["right"]["body"] == "right_foot",
        "SMPL-X source foot names are invalid",
    )
    require(
        foot_contact["target"]["left"]["body"] == "l_ankle_roll_link" and
        foot_contact["target"]["right"]["body"] == "r_ankle_roll_link",
        "BUMI3 target foot names are invalid",
    )
    contact_limits = foot_contact["constraints"]
    require(
        float(contact_limits["mesh_floor_margin_m"]) == 0.0006 and
        float(contact_limits["sole_corner_floor_margin_m"]) == 0.0027 and
        float(contact_limits["max_joint_velocity_rps"]) == 6.0 and
        float(contact_limits["max_joint_acceleration_rps2"]) == 80.0 and
        float(contact_limits["max_output_root_horizontal_velocity_mps"]) == 0.75 and
        float(contact_limits["max_root_vertical_velocity_mps"]) == 0.45 and
        float(contact_limits["max_root_linear_acceleration_mps2"]) == 3.0 and
        float(contact_limits["max_root_angular_velocity_rps"]) == 6.0 and
        float(contact_limits["max_root_angular_acceleration_rps2"]) == 20.0,
        "batch frame-shared temporal limits differ from the reviewed profile",
    )

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    require(base_id >= 0, "base_link missing")
    root_joints = [
        joint for joint in range(model.njnt)
        if int(model.jnt_bodyid[joint]) == base_id
    ]
    require(
        any(model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE and
            int(model.jnt_qposadr[joint]) == 0 for joint in root_joints),
        "base_link does not own qpos[0] freejoint",
    )
    require(model.nq >= 7, f"nq must be >= 7, got {model.nq}")
    require(data.qpos.size == model.nq, "qpos.size differs from nq")

    actuated = sum(
        model.jnt_type[joint] in (
            mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE
        )
        for joint in range(model.njnt)
    )
    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint) or ""
        for joint in range(model.njnt)
    ]
    realtime_safety = config["realtime_safety"]
    require(realtime_safety["enabled"], "realtime safety must be enabled")
    require(
        float(realtime_safety["max_joint_velocity_rps"]) == 12.0 and
        float(realtime_safety["max_arm_velocity_rps"]) == 12.0 and
        float(realtime_safety["max_joint_acceleration_rps2"]) > 0.0,
        "realtime default velocity must match bumi.py and acceleration must be positive",
    )
    joint_velocity_limits = realtime_safety["joint_velocity_limits_rps"]
    require(len(joint_velocity_limits) == 21, "velocity table must contain 21 joints")
    require(
        float(joint_velocity_limits["waist_yaw_joint"]) == 9.0,
        "waist velocity must match bumi.py: 9 rad/s",
    )
    require(
        all(float(limit) == (9.0 if name == "waist_yaw_joint" else 12.0)
            for name, limit in joint_velocity_limits.items()),
        "joint velocity table differs from bumi.py",
    )
    for side in ("left_arm_joints", "right_arm_joints"):
        require(len(realtime_safety[side]) == 4, f"{side} must contain four joints")
        require(
            all(name in joint_names for name in realtime_safety[side]),
            f"{side} contains a joint missing from XML",
        )
    qpos_joint_order = [
        joint_names[joint]
        for joint in sorted(
            range(model.njnt), key=lambda joint: int(model.jnt_qposadr[joint])
        )
        if model.jnt_type[joint] in (
            mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE
        )
    ]
    actuator_joint_order = [
        joint_names[int(model.actuator_trnid[actuator, 0])]
        for actuator in range(model.nu)
    ]
    robot_preset = json.loads(robot_preset_path.read_text())
    require(robot_preset["num_joints"] == actuated, "robot preset joint count mismatch")
    require(
        robot_preset["joint_names_mujoco_qpos_order"] == qpos_joint_order,
        "robot preset qpos joint order differs from XML",
    )
    require(
        robot_preset["joint_names_actuator_order"] == actuator_joint_order,
        "robot preset actuator order differs from XML",
    )
    require(
        robot_preset["joint_names_publish_order"] == GMT_JOINT_ORDER,
        "robot preset publish order differs from GMT policy metadata",
    )
    expected_joint_ids_map = [qpos_joint_order.index(name) for name in GMT_JOINT_ORDER]
    require(
        robot_preset["joint_ids_map"] == expected_joint_ids_map,
        "robot preset joint_ids_map does not map MuJoCo qpos to GMT order",
    )
    require(
        sorted(robot_preset["joint_ids_map"]) == list(range(actuated)),
        "joint_ids_map must be a complete 21-joint permutation",
    )
    require(robot_preset["publish_order_verified"], "publish order must be verified")
    require(
        robot_preset["default_key"] == "gmt_online_frame_bumi",
        "BUMI3 Redis key must match GMT",
    )
    print(f"PASS runtime config SHA-256: {runtime_hash}")
    print(f"PASS XML: {xml_path}")
    print(
        f"PASS model nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} actuated_joints={actuated}"
    )
    print("PASS configured robot bodies: 12/12 present")
    print("PASS SMP1 input unchanged: 14 supported, 12 consumed, wrists unused")
    print("PASS offsets finite, quaternion norms valid, scales positive")
    print("PASS bumi.py velocity table: waist=9 rad/s, other 20 joints=12 rad/s")
    print("PASS realtime acceleration and arm jump profile")
    print("PASS support-foot detector keeps true flight frames")
    print("PASS robot preset matches XML qpos/actuator order (21 joints)")
    print(f"PASS Redis publish order: MuJoCo qpos -> GMT {expected_joint_ids_map}")


if __name__ == "__main__":
    main()
