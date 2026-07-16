#!/usr/bin/env python3
"""Print the fixed, read-only SMPL-X/E1 configuration comparison."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_GMR = Path(
    "/home/weili/GMR/general_motion_retargeting/ik_configs/smplx_to_g1.json"
)
CONFIGS = (
    ("smplx_to_g1", ORIGINAL_GMR, ROOT / "config/ik_configs/smplx_to_g1.json"),
    ("smplx_to_e1", ROOT / "config/ik_configs/smplx_to_e1.json", None),
    ("manual_v3", ROOT / "config/ik_configs/smplx_to_e1_manual_v3.json", None),
    ("xrobot_to_e1", ROOT / "config/ik_configs/xrobot_to_e1.json", None),
    ("xsens_to_e1", ROOT / "config/ik_configs/xsens_to_e1.json", None),
    ("fbx_to_e1", ROOT / "config/ik_configs/fbx_to_e1.json", None),
)


def load(primary: Path, fallback: Path | None) -> tuple[Path, dict]:
    path = primary if primary.is_file() else fallback
    if path is None or not path.is_file():
        raise FileNotFoundError(primary)
    return path, json.loads(path.read_text())


def fmt(values: object) -> str:
    return json.dumps(values, separators=(",", ":"))


def print_config(label: str, path: Path, config: dict) -> None:
    print(f"\n## {label}: {path}")
    for body in config.get("ik_match_table1", {}):
        first = config["ik_match_table1"][body]
        second = config.get("ik_match_table2", {}).get(body, [first[0], 0, 0, first[3], first[4]])
        print(
            f"{first[0]:20s} -> {body:28s} "
            f"t1(pos={first[1]:g},rot={first[2]:g}) "
            f"t2(pos={second[1]:g},rot={second[2]:g}) "
            f"q={fmt(first[4])} p={fmt(first[3])}"
        )


def main() -> None:
    loaded: dict[str, dict] = {}
    for label, primary, fallback in CONFIGS:
        path, config = load(primary, fallback)
        if label == "smplx_to_g1" and path != primary:
            print(f"[note] original GMR repo absent; using verified local copy: {path}")
        loaded[label] = config
        print_config(label, path, config)

    current = loaded["smplx_to_e1"]["ik_match_table1"]
    xrobot = loaded["xrobot_to_e1"]["ik_match_table1"]
    pairs = (
        ("left_shoulder", "l_arm_shoulder_roll_link"),
        ("left_elbow", "l_arm_elbow_pitch_link"),
        ("left_wrist", "l_arm_elbow_yaw_link"),
        ("right_shoulder", "r_arm_shoulder_roll_link"),
        ("right_elbow", "r_arm_elbow_pitch_link"),
        ("right_wrist", "r_arm_elbow_yaw_link"),
    )
    print("\n## Arm quaternion differences: current SMPL-X E1 vs XRobot E1")
    for human, body in pairs:
        print(
            f"{human:16s} {body:28s} "
            f"current={fmt(current[body][4])} xrobot={fmt(xrobot[body][4])}"
        )


if __name__ == "__main__":
    main()
