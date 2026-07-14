#!/usr/bin/env python3
"""Calibrate GEM human-frame quaternion offsets against a MuJoCo neutral pose.

Run the position-only GEM server first so that
<redis-key>_raw_bones is populated, hold a stable reference pose, and execute
this script.  It averages raw human quaternions, computes

    q_offset = inverse(q_human_reference) * q_robot_neutral

and writes those offsets into both IK match tables of a copied config.

The script only updates quaternion offsets; tune rotation weights separately.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np
import redis


def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-9:
        raise ValueError("zero quaternion")
    q = q / n
    return q if q[0] >= 0 else -q


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)


def mean_quaternion(samples: list[np.ndarray]) -> np.ndarray:
    ref = q_normalize(samples[0])
    aligned = []
    for sample in samples:
        q = q_normalize(sample)
        if np.dot(q, ref) < 0:
            q = -q
        aligned.append(q)
    # Principal eigenvector of sum(q q^T).
    A = sum(np.outer(q, q) for q in aligned)
    _, vecs = np.linalg.eigh(A)
    return q_normalize(vecs[:, -1])


def collect_raw_bones(
    client: redis.Redis, key: str, duration: float, sample_hz: float
) -> dict[str, np.ndarray]:
    by_name: dict[str, list[np.ndarray]] = {}
    deadline = time.monotonic() + duration
    period = 1.0 / sample_hz

    while time.monotonic() < deadline:
        t0 = time.monotonic()
        raw = client.get(key)
        if raw:
            obj = json.loads(raw)
            for name, values in obj.items():
                if len(values) >= 7:
                    by_name.setdefault(name, []).append(
                        np.asarray(values[3:7], dtype=np.float64)
                    )
        time.sleep(max(0.0, period - (time.monotonic() - t0)))

    if not by_name:
        raise RuntimeError(f"no raw bones received from Redis key {key!r}")
    return {name: mean_quaternion(qs) for name, qs in by_name.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml", required=True)
    parser.add_argument("--base-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-key", default="mmocap_motion_frame_g1")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--sample-hz", type=float, default=30.0)
    args = parser.parse_args()

    client = redis.Redis(
        host=args.redis_host, port=args.redis_port, db=args.redis_db
    )
    raw_key = args.redis_key + "_raw_bones"
    print(f"Hold the reference pose; collecting {args.duration:.1f}s from {raw_key} ...")
    human_q = collect_raw_bones(
        client, raw_key, args.duration, args.sample_hz
    )

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    config = json.loads(Path(args.base_config).read_text())
    changed = 0
    for table_name in ("ik_match_table1", "ik_match_table2"):
        table = config.get(table_name, {})
        for robot_body, entry in table.items():
            human_body = entry[0]
            if human_body not in human_q:
                print(f"[skip] missing human bone: {human_body}")
                continue
            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, robot_body
            )
            if body_id < 0:
                print(f"[skip] missing MuJoCo body: {robot_body}")
                continue

            q_h = human_q[human_body]
            q_r = q_normalize(np.asarray(data.xquat[body_id], dtype=np.float64))
            q_off = q_normalize(q_mul(q_conj(q_h), q_r))
            entry[4] = [round(float(x), 8) for x in q_off]
            changed += 1
            print(f"[ok] {robot_body:28s} <- {human_body:20s} {entry[4]}")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + "\n")
    print(f"Wrote {output} ({changed} table entries updated).")


if __name__ == "__main__":
    main()
