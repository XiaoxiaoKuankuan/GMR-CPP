#!/usr/bin/env python3
"""Compare local Xsens base/jump configs without changing any IK config."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
TABLES = ("ik_match_table1", "ik_match_table2")
TOP_LEVEL_FIELDS = (
    "robot_root_name",
    "human_root_name",
    "ground_height",
    "human_height_assumption",
    "use_ik_match_table1",
    "use_ik_match_table2",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    return json.loads(path.read_text())


def close(a: float, b: float, tolerance: float = 1e-9) -> bool:
    return math.isclose(float(a), float(b), rel_tol=tolerance, abs_tol=tolerance)


def vector_equal(a: list[float], b: list[float]) -> bool:
    return len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))


def quaternion_relation(a: list[float], b: list[float]) -> str:
    if vector_equal(a, b):
        return "equal"
    if len(a) == len(b) and vector_equal(a, [-value for value in b]):
        return "sign-equivalent"
    return "changed"


def scalar_change(base: float, jump: float) -> dict[str, Any]:
    delta = float(jump) - float(base)
    ratio = None if close(base, 0.0) else float(jump) / float(base)
    return {
        "base": base,
        "jump": jump,
        "changed": not close(base, jump),
        "delta": delta,
        "absolute_delta": abs(delta),
        "ratio": ratio,
        "direction": 0 if close(delta, 0.0) else (1 if delta > 0.0 else -1),
    }


def compare_robot(
    name: str,
    base_path: Path,
    jump_path: Path,
) -> dict[str, Any]:
    base = load(base_path)
    jump = load(jump_path)
    result: dict[str, Any] = {
        "name": name,
        "base_path": str(base_path.resolve()),
        "jump_path": str(jump_path.resolve()),
        "base_sha256": sha256(base_path),
        "jump_sha256": sha256(jump_path),
        "top_level": [],
        "human_scale": [],
        "tables": {},
    }

    for field in TOP_LEVEL_FIELDS:
        result["top_level"].append(
            {
                "field": field,
                "base": base.get(field),
                "jump": jump.get(field),
                "changed": base.get(field) != jump.get(field),
            }
        )

    base_scales = base["human_scale_table"]
    jump_scales = jump["human_scale_table"]
    if set(base_scales) != set(jump_scales):
        raise ValueError(f"{name}: human_scale_table keys differ")
    for human_body in base_scales:
        result["human_scale"].append(
            {"human_body": human_body, **scalar_change(
                base_scales[human_body], jump_scales[human_body]
            )}
        )

    for table_name in TABLES:
        base_table = base[table_name]
        jump_table = jump[table_name]
        if set(base_table) != set(jump_table):
            raise ValueError(f"{name}: {table_name} robot body keys differ")
        rows = []
        for robot_body, base_entry in base_table.items():
            jump_entry = jump_table[robot_body]
            if base_entry[0] != jump_entry[0]:
                raise ValueError(
                    f"{name}: {table_name}/{robot_body} human body changed"
                )
            position_relation = (
                "equal" if vector_equal(base_entry[3], jump_entry[3]) else "changed"
            )
            rotation_relation = quaternion_relation(base_entry[4], jump_entry[4])
            rows.append(
                {
                    "robot_body": robot_body,
                    "human_body": base_entry[0],
                    "position_weight": scalar_change(base_entry[1], jump_entry[1]),
                    "rotation_weight": scalar_change(base_entry[2], jump_entry[2]),
                    "position_offset": {
                        "base": base_entry[3],
                        "jump": jump_entry[3],
                        "relation": position_relation,
                        "changed": position_relation != "equal",
                    },
                    "rotation_offset": {
                        "base": base_entry[4],
                        "jump": jump_entry[4],
                        "relation": rotation_relation,
                        "changed": rotation_relation == "changed",
                    },
                }
            )
        result["tables"][table_name] = rows
    return result


def scalar_index(robot: dict[str, Any]) -> dict[tuple[str, str, str], dict[str, Any]]:
    index: dict[tuple[str, str, str], dict[str, Any]] = {}
    for row in robot["human_scale"]:
        index[("human_scale", row["human_body"], "scale")] = row
    for table_name, rows in robot["tables"].items():
        for row in rows:
            index[(table_name, row["human_body"], "position_weight")] = row[
                "position_weight"
            ]
            index[(table_name, row["human_body"], "rotation_weight")] = row[
                "rotation_weight"
            ]
    return index


def cross_robot_rules(g1: dict[str, Any], e1: dict[str, Any]) -> list[dict[str, Any]]:
    g1_index = scalar_index(g1)
    e1_index = scalar_index(e1)
    if set(g1_index) != set(e1_index):
        missing_g1 = sorted(set(e1_index) - set(g1_index))
        missing_e1 = sorted(set(g1_index) - set(e1_index))
        raise ValueError(
            f"semantic Xsens targets differ: missing_g1={missing_g1}, "
            f"missing_e1={missing_e1}"
        )

    rules = []
    for key in sorted(g1_index):
        g1_change = g1_index[key]
        e1_change = e1_index[key]
        both_changed = g1_change["changed"] and e1_change["changed"]
        same_direction = (
            both_changed
            and g1_change["direction"] == e1_change["direction"]
        )
        exact_same_change = (
            both_changed and close(g1_change["delta"], e1_change["delta"])
        )
        if exact_same_change:
            classification = "common-exact"
        elif same_direction:
            classification = "common-direction-e1-ratio"
        elif g1_change["changed"] != e1_change["changed"]:
            changed_robot = "g1" if g1_change["changed"] else "e1"
            classification = f"robot-specific-{changed_robot}"
        elif both_changed:
            classification = "conflicting-direction"
        else:
            classification = "unchanged"
        rules.append(
            {
                "scope": key[0],
                "human_body": key[1],
                "field": key[2],
                "g1": g1_change,
                "e1": e1_change,
                "both_changed": both_changed,
                "same_direction": same_direction,
                "exact_same_change": exact_same_change,
                "robot_specific": classification.startswith("robot-specific"),
                "classification": classification,
            }
        )
    return rules


def arrow(base: Any, jump: Any) -> str:
    return str(base) if base == jump else f"{base} → {jump}"


def markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Xsens base → jump 配置逐字段差异",
        "",
        "四元数比较将 `q` 与 `-q` 视为同一旋转。",
        "",
    ]
    for robot_name in ("g1", "e1"):
        robot = report["robots"][robot_name]
        lines.extend(
            [
                f"## {robot_name.upper()}",
                "",
                f"- base: `{robot['base_path']}`",
                f"- jump: `{robot['jump_path']}`",
                "",
                "### 顶层字段",
                "",
                "| 字段 | base → jump | 是否改变 |",
                "|---|---:|---|",
            ]
        )
        for row in robot["top_level"]:
            lines.append(
                f"| `{row['field']}` | `{arrow(row['base'], row['jump'])}` | "
                f"{'是' if row['changed'] else '否'} |"
            )
        lines.extend(
            [
                "",
                "### Human scale",
                "",
                "| Human body | base → jump | Δ | 比例 |",
                "|---|---:|---:|---:|",
            ]
        )
        for row in robot["human_scale"]:
            ratio = "—" if row["ratio"] is None else f"{row['ratio']:.6g}"
            lines.append(
                f"| `{row['human_body']}` | `{arrow(row['base'], row['jump'])}` | "
                f"{row['delta']:.6g} | {ratio} |"
            )
        for table_name in TABLES:
            lines.extend(
                [
                    "",
                    f"### `{table_name}`",
                    "",
                    "| Robot body | Human body | pos base→jump | pos Δ/ratio | "
                    "rot base→jump | rot Δ/ratio | position offset | rotation offset |",
                    "|---|---|---:|---:|---:|---:|---|---|",
                ]
            )
            for row in robot["tables"][table_name]:
                pos = row["position_weight"]
                rot = row["rotation_weight"]
                pos_ratio = "—" if pos["ratio"] is None else f"{pos['ratio']:.6g}"
                rot_ratio = "—" if rot["ratio"] is None else f"{rot['ratio']:.6g}"
                lines.append(
                    f"| `{row['robot_body']}` | `{row['human_body']}` | "
                    f"`{arrow(pos['base'], pos['jump'])}` | "
                    f"{pos['delta']:.6g} / {pos_ratio} | "
                    f"`{arrow(rot['base'], rot['jump'])}` | "
                    f"{rot['delta']:.6g} / {rot_ratio} | "
                    f"{row['position_offset']['relation']} | "
                    f"{row['rotation_offset']['relation']} |"
                )

    lines.extend(
        [
            "",
            "## G1/E1 交叉规律",
            "",
            "| 范围 | Human body | 字段 | G1 Δ/ratio | E1 Δ/ratio | 同向 | 分类 |",
            "|---|---|---|---:|---:|---|---|",
        ]
    )
    for rule in report["cross_robot_rules"]:
        if not (rule["g1"]["changed"] or rule["e1"]["changed"]):
            continue
        def change_text(value: dict[str, Any]) -> str:
            ratio = "—" if value["ratio"] is None else f"{value['ratio']:.6g}"
            return f"{value['delta']:.6g} / {ratio}"
        lines.append(
            f"| `{rule['scope']}` | `{rule['human_body']}` | `{rule['field']}` | "
            f"{change_text(rule['g1'])} | {change_text(rule['e1'])} | "
            f"{'是' if rule['same_direction'] else '否'} | "
            f"`{rule['classification']}` |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g1-base", type=Path, default=ROOT / "config/ik_configs/xsens_to_g1.json")
    parser.add_argument("--g1-jump", type=Path, default=ROOT / "config/ik_configs/xsens_to_g1_jump.json")
    parser.add_argument("--e1-base", type=Path, default=ROOT / "config/ik_configs/xsens_to_e1.json")
    parser.add_argument("--e1-jump", type=Path, default=ROOT / "config/ik_configs/xsens_to_e1_jump.json")
    parser.add_argument("--json-output", type=Path, default=ROOT / "build/xsens_jump_diff.json")
    parser.add_argument("--markdown-output", type=Path)
    args = parser.parse_args()

    paths = (args.g1_base, args.g1_jump, args.e1_base, args.e1_jump)
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        found = sorted(str(path) for path in (ROOT / "config/ik_configs").glob("*xsens*.json"))
        raise SystemExit(
            "Missing required semantic base/jump config(s): " + str(missing)
            + "\nActually found:\n" + "\n".join(found)
        )

    g1 = compare_robot("g1", args.g1_base, args.g1_jump)
    e1 = compare_robot("e1", args.e1_base, args.e1_jump)
    report = {
        "schema_version": 1,
        "quaternion_sign_equivalent": True,
        "robots": {"g1": g1, "e1": e1},
        "cross_robot_rules": cross_robot_rules(g1, e1),
    }
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    rendered = markdown(report)
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(rendered)
    print(rendered, end="")
    print(f"machine-readable report: {args.json_output.resolve()}")


if __name__ == "__main__":
    main()
