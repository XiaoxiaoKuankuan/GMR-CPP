#!/usr/bin/env python3
"""Publish OMG Unitree G1 qpos36 frames to Redis as documented JSON.

Modes:
  --input motion.npy/.npz       deterministic 30 Hz replay
  --stdin                       qpos36 arrays/objects as JSON lines
  --planner-connect tcp://...   consume OMG realtime planner chunks
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np


class RedisConnection:
    def __init__(self, host: str, port: int, db: int) -> None:
        self._socket = socket.create_connection((host, port), timeout=2.0)
        self._file = self._socket.makefile("rb")
        if db:
            self.command("SELECT", str(db))

    @staticmethod
    def _encode(parts: tuple[str, ...]) -> bytes:
        payload = [f"*{len(parts)}\r\n".encode()]
        for part in parts:
            value = part.encode("utf-8")
            payload.extend((f"${len(value)}\r\n".encode(), value, b"\r\n"))
        return b"".join(payload)

    def command(self, *parts: str) -> bytes:
        self._socket.sendall(self._encode(tuple(parts)))
        prefix = self._file.read(1)
        if prefix in {b"+", b"-", b":"}:
            response = self._file.readline().rstrip(b"\r\n")
            if prefix == b"-":
                raise RuntimeError(f"Redis error: {response.decode(errors='replace')}")
            return response
        if prefix == b"$":
            length = int(self._file.readline())
            if length < 0:
                return b""
            response = self._file.read(length)
            self._file.read(2)
            return response
        raise RuntimeError(f"unexpected Redis response prefix: {prefix!r}")

    def set_json(self, key: str, payload: str, ttl_ms: int) -> None:
        if ttl_ms > 0:
            self.command("SET", key, payload, "PX", str(ttl_ms))
        else:
            self.command("SET", key, payload)

    def close(self) -> None:
        self._file.close()
        self._socket.close()


def load_qpos(path: str | Path) -> tuple[np.ndarray, float | None]:
    motion_path = Path(path).expanduser()
    loaded_fps: float | None = None
    if motion_path.suffix == ".npy":
        qpos = np.load(motion_path, allow_pickle=False)
    elif motion_path.suffix == ".npz":
        with np.load(motion_path, allow_pickle=False) as data:
            for key in ("qpos_36", "pred_qpos_36", "executed_qpos_36", "qpos"):
                if key in data:
                    qpos = np.asarray(data[key])
                    break
            else:
                raise KeyError(f"no qpos36 array in {motion_path}")
            if "fps" in data:
                loaded_fps = float(np.asarray(data["fps"]).reshape(-1)[0])
    else:
        raise ValueError("--input must be .npy or .npz")
    qpos = np.asarray(qpos, dtype=np.float32)
    if qpos.ndim == 3 and qpos.shape[0] == 1:
        qpos = qpos[0]
    if qpos.ndim != 2 or qpos.shape[1] != 36 or qpos.shape[0] == 0:
        raise ValueError(f"expected qpos shape (T,36), got {qpos.shape}")
    if not np.isfinite(qpos).all():
        raise ValueError("qpos contains non-finite values")
    return qpos, loaded_fps


def validate_frame(qpos: Any) -> np.ndarray:
    frame = np.asarray(qpos, dtype=np.float64).reshape(-1)
    if frame.shape != (36,) or not np.isfinite(frame).all():
        raise ValueError("each OMG frame must contain 36 finite qpos values")
    norm = float(np.linalg.norm(frame[3:7]))
    if norm < 1e-10:
        raise ValueError("root quaternion has zero norm")
    frame[3:7] /= norm
    return frame


def frame_json(qpos: Any, timestamp: float) -> str:
    frame = validate_frame(qpos)
    return json.dumps(
        {
            "timestamp": float(timestamp),
            "root_pos": frame[:3].tolist(),
            "root_quat": frame[3:7].tolist(),
            "joints": frame[7:].tolist(),
        },
        separators=(",", ":"),
        allow_nan=False,
    )


def publish_frame(redis: RedisConnection, key: str, ttl_ms: int,
                  qpos: Any, timestamp: float) -> None:
    redis.set_json(key, frame_json(qpos, timestamp), ttl_ms)


def sleep_until(deadline: float) -> None:
    remaining = deadline - time.monotonic()
    if remaining > 0.0:
        time.sleep(remaining)


def replay_input(args: argparse.Namespace, redis: RedisConnection) -> None:
    qpos, loaded_fps = load_qpos(args.input)
    fps = float(args.fps if args.fps is not None else loaded_fps or 30.0)
    sent = 0
    while True:
        start = time.monotonic()
        for index, frame in enumerate(qpos):
            publish_frame(redis, args.redis_key, args.ttl_ms, frame,
                          time.monotonic())
            sent += 1
            sleep_until(start + (index + 1) / fps)
        if not args.loop:
            break
    print(f"[OMG Redis] input={Path(args.input).resolve()} frames={sent} fps={fps}")


def stream_stdin(args: argparse.Namespace, redis: RedisConnection) -> None:
    sent = 0
    for line in sys.stdin:
        value = json.loads(line)
        if isinstance(value, dict) and {"root_pos", "root_quat", "joints"} <= value.keys():
            payload = dict(value)
            payload.setdefault("timestamp", time.monotonic())
            qpos = [*payload["root_pos"], *payload["root_quat"], *payload["joints"]]
        elif isinstance(value, dict) and "qpos_36" in value:
            qpos = value["qpos_36"]
        else:
            qpos = value
        publish_frame(redis, args.redis_key, args.ttl_ms, qpos, time.monotonic())
        sent += 1
    print(f"[OMG Redis] stdin frames={sent}")


def stream_planner(args: argparse.Namespace, redis: RedisConnection) -> None:
    try:
        from omg.realtime.motion_buffer import ExecutedHistoryBuffer
        from omg.realtime.orin_client import (
            RealtimeOrinBufferClient,
            RealtimeOrinBufferClientConfig,
        )
        from omg.tracking.holomotion.reference import resample_qpos
    except ImportError as error:
        raise RuntimeError(
            "planner mode requires OMG: export PYTHONPATH=/home/weili/OMG/src"
        ) from error

    seed, loaded_fps = load_qpos(args.seed_motion)
    seed_fps = float(args.seed_fps if args.seed_fps is not None else loaded_fps or 30.0)
    fps = float(args.fps or 30.0)
    history = ExecutedHistoryBuffer(
        target_fps=float(args.history_fps), max_frames=int(args.history_frames)
    )
    seed_history = resample_qpos(seed, source_fps=seed_fps,
                                 target_fps=float(args.history_fps))
    history.append(seed_history[-args.history_frames :], fps=float(args.history_fps))
    client = RealtimeOrinBufferClient(
        RealtimeOrinBufferClientConfig(
            connect=args.planner_connect,
            tracker_fps=fps,
            request_timeout_ms=int(args.timeout_ms),
        )
    )
    cursor = 0
    condition_index = 0
    condition_session_id = uuid.uuid4().hex
    start = time.monotonic()
    pending = False

    def request_metadata(index: int) -> dict[str, Any]:
        return {
            "condition_sequence": args.condition_sequence,
            "condition_index": index,
            "condition_session_id": condition_session_id,
            "condition_source": "condition_sequence",
            "audio_fps": float(args.audio_fps),
            "tracker_fps": fps,
            "audio_type": args.audio_type,
        }

    def append_response(response: Any) -> None:
        client.append_response(response, current_tracker_frame=cursor)
        print(
            f"[OMG Redis replan] plan={response.plan_id} cursor={cursor} "
            f"frames={response.qpos_36.shape[0]} prompt={response.prompt!r}",
            flush=True,
        )

    def request_initial() -> None:
        nonlocal condition_index
        response = client.request_plan(
            tracker_frame=cursor,
            qpos_36_history=history.history(args.history_frames),
            history_fps=float(args.history_fps),
            metadata=request_metadata(condition_index),
        )
        append_response(response)
        condition_index += 1

    def begin_replan() -> None:
        nonlocal condition_index, pending
        client.begin_request(
            tracker_frame=cursor,
            qpos_36_history=history.history(args.history_frames),
            history_fps=float(args.history_fps),
            metadata=request_metadata(condition_index),
        )
        condition_index += 1
        pending = True

    try:
        request_initial()
        start = time.monotonic()
        while args.num_frames <= 0 or cursor < args.num_frames:
            if pending:
                response = client.poll_response(timeout_ms=0)
                if response is not None:
                    append_response(response)
                    pending = False
            if (not pending and
                    client.buffer.remaining(cursor) <= args.replan_remaining_frames):
                begin_replan()
            if client.buffer.remaining(cursor) <= 0:
                if not pending:
                    raise RuntimeError(f"motion buffer underrun at frame {cursor}")
                response = client.poll_response(timeout_ms=int(args.timeout_ms))
                if response is None:
                    raise RuntimeError("timed out waiting for pending OMG plan")
                append_response(response)
                pending = False
                start = time.monotonic() - cursor / fps
            frame = client.buffer.slice(cursor, 1)[0]
            publish_frame(redis, args.redis_key, args.ttl_ms, frame,
                          time.monotonic())
            history.append(frame.reshape(1, 36), fps=fps)
            cursor += 1
            sleep_until(start + cursor / fps)
    finally:
        client.close()
    print(f"[OMG Redis] planner frames={cursor} fps={fps}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--input", help="OMG qpos36 .npy/.npz to replay")
    mode.add_argument("--stdin", action="store_true", help="read JSON lines")
    mode.add_argument("--planner-connect", help="OMG realtime planner URI")
    parser.add_argument("--redis-host", default="127.0.0.1")
    parser.add_argument("--redis-port", type=int, default=6379)
    parser.add_argument("--redis-db", type=int, default=0)
    parser.add_argument("--redis-key", default="omg_online_frame_g1")
    parser.add_argument("--ttl-ms", type=int, default=500)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--seed-motion")
    parser.add_argument("--seed-fps", type=float, default=None)
    parser.add_argument("--history-fps", type=float, default=30.0)
    parser.add_argument("--history-frames", type=int, default=10)
    parser.add_argument("--replan-remaining-frames", type=int, default=40)
    parser.add_argument("--num-frames", type=int, default=0,
                        help="planner frames; 0 runs continuously")
    parser.add_argument("--timeout-ms", type=int, default=120000)
    parser.add_argument("--condition-sequence")
    parser.add_argument("--audio-type", choices=("audio", "feature"), default="audio")
    parser.add_argument("--audio-fps", type=float, default=30.0)
    args = parser.parse_args()
    if args.planner_connect:
        if not args.seed_motion or not args.condition_sequence:
            parser.error("planner mode requires --seed-motion and --condition-sequence")
    if args.fps is not None and args.fps <= 0.0:
        parser.error("--fps must be positive")
    return args


def main() -> None:
    args = parse_args()
    redis = RedisConnection(args.redis_host, args.redis_port, args.redis_db)
    print(
        f"[OMG Redis] {args.redis_host}:{args.redis_port}/{args.redis_db} "
        f"key={args.redis_key} ttl={args.ttl_ms}ms",
        flush=True,
    )
    try:
        if args.input:
            replay_input(args, redis)
        elif args.stdin:
            stream_stdin(args, redis)
        else:
            stream_planner(args, redis)
    except KeyboardInterrupt:
        print("[OMG Redis] interrupted", flush=True)
    finally:
        redis.close()


if __name__ == "__main__":
    main()
