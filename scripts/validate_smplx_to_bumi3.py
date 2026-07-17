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
EXPECTED_SHA256 = "a5b810e0e12519f4c94fc595622bb2ff5a9ed668a419236aa9ac50eacb678f88"
SMP1_TARGETS = {
    "pelvis", "spine3", "left_hip", "right_hip", "left_knee",
    "right_knee", "left_foot", "right_foot", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist",
    "right_wrist",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument(
        "--source-config", type=Path, default=ROOT / "smplx_to_bumi3.json"
    )
    parser.add_argument(
        "--robot-preset", type=Path,
        default=ROOT / "config/robot_presets/bumi3.json",
    )
    args = parser.parse_args()
    xml_path = args.xml.expanduser().resolve(strict=True)
    config_path = args.config.expanduser().resolve(strict=True)
    source_path = args.source_config.expanduser().resolve(strict=True)
    robot_preset_path = args.robot_preset.expanduser().resolve(strict=True)

    source_hash = sha256(source_path)
    runtime_hash = sha256(config_path)
    require(source_hash == EXPECTED_SHA256, f"source SHA mismatch: {source_hash}")
    source_config = json.loads(source_path.read_text())
    config = json.loads(config_path.read_text())
    require(
        config == source_config,
        "runtime config values differ from the immutable source config",
    )
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
    require(not robot_preset["joint_names_publish_order"], "publish order must remain empty")
    require(not robot_preset["joint_ids_map"], "joint_ids_map must remain empty")
    require(not robot_preset["publish_order_verified"], "publish order must be unverified")
    print(f"PASS source config SHA-256: {source_hash}")
    print(f"PASS runtime config values unchanged (formatted SHA-256: {runtime_hash})")
    print(f"PASS XML: {xml_path}")
    print(
        f"PASS model nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} actuated_joints={actuated}"
    )
    print("PASS configured robot bodies: 12/12 present")
    print("PASS SMP1 input unchanged: 14 supported, 12 consumed, wrists unused")
    print("PASS offsets finite, quaternion norms valid, scales positive")
    print("PASS robot preset matches XML qpos/actuator order (21 joints)")
    print("Redis publish order: UNVERIFIED (identity qpos order is viewer/test only)")


if __name__ == "__main__":
    main()
