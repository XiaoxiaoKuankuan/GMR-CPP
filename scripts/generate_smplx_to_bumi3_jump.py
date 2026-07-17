#!/usr/bin/env python3
"""Generate a fixed BUMI3 jump JSON from the audited Xsens jump diff."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TABLES = ("ik_match_table1", "ik_match_table2")
SMPL_TO_XSENS = {
    "pelvis": "Pelvis",
    "spine3": "Chest",
    "left_hip": "Left_UpperLeg",
    "right_hip": "Right_UpperLeg",
    "left_knee": "Left_LowerLeg",
    "right_knee": "Right_LowerLeg",
    "left_foot": "Left_Foot",
    "right_foot": "Right_Foot",
    "left_shoulder": "Left_UpperArm",
    "right_shoulder": "Right_UpperArm",
    "left_elbow": "Left_Forearm",
    "right_elbow": "Right_Forearm",
}


def load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def normalized_number(value: float) -> int | float:
    rounded = round(value)
    return int(rounded) if math.isclose(value, rounded, abs_tol=1e-12) else value


def rule_index(report: dict[str, Any]) -> dict[tuple[str, str, str], dict[str, Any]]:
    return {
        (rule["scope"], rule["human_body"], rule["field"]): rule
        for rule in report["cross_robot_rules"]
    }


def validate_fixed_config(base: dict[str, Any], jump: dict[str, Any]) -> None:
    for field in (
        "robot_root_name",
        "human_root_name",
        "ground_height",
        "human_height_assumption",
        "use_ik_match_table1",
        "use_ik_match_table2",
        "human_scale_table",
    ):
        if jump[field] != base[field]:
            raise ValueError(f"protected field changed: {field}")

    if any(not math.isfinite(float(value)) or float(value) <= 0.0
           for value in jump["human_scale_table"].values()):
        raise ValueError("all human scales must be finite and positive")

    for table_name in TABLES:
        if set(jump[table_name]) != set(base[table_name]):
            raise ValueError(f"robot body mapping changed in {table_name}")
        for robot_body, entry in jump[table_name].items():
            base_entry = base[table_name][robot_body]
            if entry[0] != base_entry[0]:
                raise ValueError(f"human mapping changed: {table_name}/{robot_body}")
            if entry[3] != base_entry[3]:
                raise ValueError(f"position offset changed: {table_name}/{robot_body}")
            if entry[4] != base_entry[4]:
                raise ValueError(f"rotation offset changed: {table_name}/{robot_body}")
            if not all(math.isfinite(float(value)) for value in entry[3]):
                raise ValueError(f"non-finite position offset: {table_name}/{robot_body}")
            norm = math.sqrt(sum(float(value) ** 2 for value in entry[4]))
            if not math.isclose(norm, 1.0, rel_tol=1e-6, abs_tol=1e-6):
                raise ValueError(f"invalid quaternion: {table_name}/{robot_body}")

    pelvis_weights = [
        entry[1]
        for table_name in TABLES
        for entry in jump[table_name].values()
        if entry[0] == "pelvis"
    ]
    if not pelvis_weights or not any(float(weight) > 0.0 for weight in pelvis_weights):
        raise ValueError("pelvis world-position task must remain enabled")
    for foot in ("left_foot", "right_foot"):
        if not any(entry[0] == foot for table in TABLES
                   for entry in jump[table].values()):
            raise ValueError(f"required foot target missing: {foot}")

    symmetric_pairs = (
        ("left_hip", "right_hip"),
        ("left_knee", "right_knee"),
        ("left_foot", "right_foot"),
        ("left_shoulder", "right_shoulder"),
        ("left_elbow", "right_elbow"),
    )
    for table_name in TABLES:
        by_human = {entry[0]: entry for entry in jump[table_name].values()}
        for left, right in symmetric_pairs:
            if by_human[left][1:3] != by_human[right][1:3]:
                raise ValueError(
                    f"asymmetric weights: {table_name}/{left}/{right}"
                )


def generate(base: dict[str, Any], report: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if report.get("schema_version") != 1:
        raise ValueError("unsupported xsens jump diff schema")
    rules = rule_index(report)
    jump = copy.deepcopy(base)
    applied: list[dict[str, Any]] = []

    # Only common table2 position-weight changes are transferred. Scale and all
    # offsets/rotations remain the already validated SMPL-X/BUMI3 values.
    table_name = "ik_match_table2"
    for robot_body, entry in jump[table_name].items():
        smpl_name = entry[0]
        xsens_name = SMPL_TO_XSENS[smpl_name]
        rule = rules[(table_name, xsens_name, "position_weight")]
        if rule["classification"] not in (
            "common-exact",
            "common-direction-e1-ratio",
        ):
            continue
        ratio = rule["e1"]["ratio"]
        if ratio is None or not math.isfinite(float(ratio)) or float(ratio) <= 0.0:
            raise ValueError(
                f"no valid E1 ratio for common rule {table_name}/{xsens_name}"
            )
        base_weight = entry[1]
        jump_weight = normalized_number(float(base_weight) * float(ratio))
        if jump_weight == base_weight:
            continue
        entry[1] = jump_weight
        applied.append(
            {
                "table": table_name,
                "robot_body": robot_body,
                "smpl_body": smpl_name,
                "xsens_body": xsens_name,
                "field": "position_weight",
                "base": base_weight,
                "jump": jump_weight,
                "e1_ratio": ratio,
                "evidence": rule["classification"],
            }
        )

    if not applied:
        raise ValueError("no common G1/E1 jump weight rule was applicable")
    validate_fixed_config(base, jump)
    return jump, applied


def compact_config(config: dict[str, Any]) -> str:
    lines = ["{"]
    keys = list(config)
    for key_index, key in enumerate(keys):
        value = config[key]
        suffix = "," if key_index + 1 < len(keys) else ""
        if key == "human_scale_table":
            lines.append(f'  "{key}": {{')
            items = list(value.items())
            for index, (name, scale) in enumerate(items):
                comma = "," if index + 1 < len(items) else ""
                lines.append(f'    "{name}": {json.dumps(scale)}{comma}')
            lines.append(f"  }}{suffix}")
        elif key in TABLES:
            lines.append(f'  "{key}": {{')
            items = list(value.items())
            for index, (body, entry) in enumerate(items):
                comma = "," if index + 1 < len(items) else ""
                encoded = json.dumps(entry, ensure_ascii=False, separators=(", ", ": "))
                lines.append(f'    "{body}": {encoded}{comma}')
            lines.append(f"  }}{suffix}")
        else:
            encoded = json.dumps(value, ensure_ascii=False)
            lines.append(f'  "{key}": {encoded}{suffix}')
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=ROOT / "config/ik_configs/smplx_to_bumi3.json")
    parser.add_argument("--diff", type=Path, default=ROOT / "build/xsens_jump_diff.json")
    parser.add_argument("--output", type=Path, default=ROOT / "config/ik_configs/smplx_to_bumi3_jump.json")
    parser.add_argument("--check", action="store_true", help="verify the existing output without rewriting it")
    args = parser.parse_args()

    base = load(args.base)
    report = load(args.diff)
    jump, applied = generate(base, report)
    rendered = compact_config(jump)
    if args.check:
        if not args.output.is_file() or args.output.read_text() != rendered:
            raise SystemExit(f"generated output is stale: {args.output}")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)

    for change in applied:
        print(
            f"{change['table']} {change['robot_body']} "
            f"{change['field']}: {change['base']} -> {change['jump']} "
            f"(E1 ratio={change['e1_ratio']}, {change['evidence']})"
        )
    print(("verified" if args.check else "wrote") + f": {args.output.resolve()}")


if __name__ == "__main__":
    main()
