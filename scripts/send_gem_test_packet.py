#!/usr/bin/env python3
"""Send a synthetic 14-bone GEM1 stream for network/GMR smoke tests."""

from __future__ import annotations

import argparse
import socket
import struct
import time


BONE_NAMES = (
    "Pelvis",
    "Chest",
    "Left_UpperLeg",
    "Right_UpperLeg",
    "Left_LowerLeg",
    "Right_LowerLeg",
    "Left_Foot",
    "Right_Foot",
    "Left_UpperArm",
    "Right_UpperArm",
    "Left_Forearm",
    "Right_Forearm",
    "Left_Hand",
    "Right_Hand",
)

BONES = (
    # px, py, pz, qw, qx, qy, qz — already normalized to GMR Z-up.
    (0.00, 0.00, 0.90, 1, 0, 0, 0),
    (0.00, 0.00, 1.20, 1, 0, 0, 0),
    (0.10, 0.00, 0.85, 1, 0, 0, 0),
    (-0.10, 0.00, 0.85, 1, 0, 0, 0),
    (0.10, 0.00, 0.50, 1, 0, 0, 0),
    (-0.10, 0.00, 0.50, 1, 0, 0, 0),
    (0.10, 0.05, 0.08, 1, 0, 0, 0),
    (-0.10, 0.05, 0.08, 1, 0, 0, 0),
    (0.25, 0.00, 1.25, 1, 0, 0, 0),
    (-0.25, 0.00, 1.25, 1, 0, 0, 0),
    (0.50, 0.00, 1.25, 1, 0, 0, 0),
    (-0.50, 0.00, 1.25, 1, 0, 0, 0),
    (0.72, 0.00, 1.25, 1, 0, 0, 0),
    (-0.72, 0.00, 1.25, 1, 0, 0, 0),
)

HEADER = struct.Struct("<4sHHIQ")
PAYLOAD = struct.Struct("<98f")
PACKET_BYTES = HEADER.size + PAYLOAD.size
assert len(BONES) == len(BONE_NAMES) == 14
assert PACKET_BYTES == 412


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send synthetic GEM1 T-pose packets (not a motion-quality test)"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7001)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument(
        "--seconds", type=float, default=0.0,
        help="stop after N seconds; 0 runs until Ctrl+C",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.hz <= 0.0:
        parser.error("--hz must be > 0")
    if args.seconds < 0.0:
        parser.error("--seconds must be >= 0")
    return args


def main() -> None:
    args = parse_args()
    values = [float(value) for bone in BONES for value in bone]
    payload = PAYLOAD.pack(*values)
    socket_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    sent = 0
    period = 1.0 / args.hz
    started = time.monotonic()
    next_tick = started

    print(
        f"[GEM test sender] destination={args.host}:{args.port} "
        f"hz={args.hz:.1f} packet bytes={PACKET_BYTES}"
    )
    try:
        while args.seconds <= 0.0 or time.monotonic() - started < args.seconds:
            packet = HEADER.pack(
                b"GEM1", 1, len(BONE_NAMES), sequence, time.monotonic_ns()
            ) + payload
            socket_out.sendto(packet, (args.host, args.port))
            sequence = (sequence + 1) & 0xFFFFFFFF
            sent += 1
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\n[GEM test sender] interrupted")
    finally:
        socket_out.close()

    elapsed = max(time.monotonic() - started, 1e-9)
    print(f"[GEM test sender] sent={sent} average={sent / elapsed:.1f}Hz")


if __name__ == "__main__":
    main()
