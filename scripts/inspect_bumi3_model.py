#!/usr/bin/env python3
"""Read-only inspection of the BUMI3 MuJoCo model and IK body references."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

try:
    import mujoco
except ModuleNotFoundError as error:  # pragma: no cover - environment guidance
    raise SystemExit(
        "mujoco Python package is required; use /home/weili/GENMO/.venv/bin/python"
    ) from error


ROOT = Path(__file__).resolve().parents[1]


def names(model: mujoco.MjModel, object_type: mujoco.mjtObj, count: int) -> list[str]:
    return [mujoco.mj_id2name(model, object_type, index) or "" for index in range(count)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--xml", type=Path, default=ROOT / "assets/bumi3/mjcf/bumi3.xml"
    )
    parser.add_argument(
        "--config", type=Path,
        default=ROOT / "config/ik_configs/smplx_to_bumi3.json",
    )
    args = parser.parse_args()
    xml_path = args.xml.expanduser().resolve(strict=True)
    config_path = args.config.expanduser().resolve(strict=True)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    config = json.loads(config_path.read_text())

    body_names = names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)
    joint_names = names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuator_names = names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    duplicate_bodies = sorted(
        name for name, count in Counter(body_names).items() if name and count > 1
    )
    referenced = sorted(
        set(config["ik_match_table1"]) | set(config["ik_match_table2"])
    )
    missing = sorted(set(referenced) - set(body_names))

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    freejoints = []
    for joint_id, joint_name in enumerate(joint_names):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            freejoints.append(
                {
                    "name": joint_name,
                    "body": body_names[int(model.jnt_bodyid[joint_id])],
                    "qpos_address": int(model.jnt_qposadr[joint_id]),
                }
            )

    print(f"XML: {xml_path}")
    print(f"config: {config_path}")
    print(
        f"nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} njnt={model.njnt} qpos.size={data.qpos.size}"
    )
    print(f"base_link body id: {base_id}")
    print(f"freejoint: {freejoints}")
    print(f"duplicate body names: {duplicate_bodies or 'none'}")
    print(f"configured robot bodies: {len(referenced)}")
    print(f"missing configured robot bodies: {missing or 'none'}")

    print("\nJoint qpos order:")
    for joint_id, joint_name in enumerate(joint_names):
        body_id = int(model.jnt_bodyid[joint_id])
        print(
            f"  {joint_id:2d}: {joint_name:<26} "
            f"qpos={int(model.jnt_qposadr[joint_id]):2d} "
            f"dof={int(model.jnt_dofadr[joint_id]):2d} "
            f"body={body_names[body_id]}"
        )

    print("\nActuator order:")
    for actuator_id, actuator_name in enumerate(actuator_names):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        print(f"  {actuator_id:2d}: {actuator_name} -> {joint_names[joint_id]}")

    print("\nBody names:")
    for body_id, body_name in enumerate(body_names):
        print(f"  {body_id:2d}: {body_name}")


if __name__ == "__main__":
    main()
