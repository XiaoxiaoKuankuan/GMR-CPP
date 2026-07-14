#!/usr/bin/env python3
import argparse
import csv
import os
import signal
import struct
import sys
import time
from collections import deque
from datetime import datetime

import redis


def parse_args():
    ap = argparse.ArgumentParser(
        description="Record root_pos_w from the GMR Redis Stream and plot it live."
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=6379)
    ap.add_argument("--db", type=int, default=0)
    ap.add_argument("--key", default="mmocap_motion_frame_g1",
                    help="Redis latest key; stream defaults to <key>:stream")
    ap.add_argument("--stream-key", default="",
                    help="Override stream key, e.g. mmocap_motion_frame_g1:stream")
    ap.add_argument("--dof", type=int, default=29)
    ap.add_argument("--from-start", action="store_true",
                    help="Read stream from 0-0 instead of only new frames")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Stop after N seconds; 0 means run until Ctrl-C")
    ap.add_argument("--window", type=float, default=20.0,
                    help="Live plot time window in seconds")
    ap.add_argument("--output", default="",
                    help="CSV path; default logs/redis_root_pos_<timestamp>.csv")
    ap.add_argument("--no-plot", action="store_true",
                    help="Record CSV only")
    return ap.parse_args()


def decode_frame(blob, dof):
    # GMR frame protocol:
    # [t, root_pos_w(3), root_quat_wxyz(4), root_lin_vel_w(3),
    #  root_ang_vel_w(3), joint_pos(dof)] as float32 little-endian.
    n_floats = 1 + 3 + 4 + 3 + 3 + dof
    expected_bytes = n_floats * 4
    if len(blob) != expected_bytes:
        raise ValueError(f"blob size {len(blob)} != expected {expected_bytes}")
    vals = struct.unpack_from("<" + "f" * n_floats, blob)
    payload_t = vals[0]
    root_x, root_y, root_z = vals[1], vals[2], vals[3]
    return payload_t, root_x, root_y, root_z


class LivePlot:
    def __init__(self, window_s):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.window_s = window_s
        self.t = deque()
        self.x = deque()
        self.y = deque()
        self.z = deque()

        plt.ion()
        self.fig, (self.ax_xyz, self.ax_xy) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.canvas.manager.set_window_title("Redis root_pos_w")
        self.line_x, = self.ax_xyz.plot([], [], label="x")
        self.line_y, = self.ax_xyz.plot([], [], label="y")
        self.line_z, = self.ax_xyz.plot([], [], label="z")
        self.xy_line, = self.ax_xy.plot([], [], label="xy")
        self.xy_dot, = self.ax_xy.plot([], [], "o", markersize=4, label="latest")

        self.ax_xyz.set_title("root_pos_w from Redis Stream")
        self.ax_xyz.set_xlabel("time since start (s)")
        self.ax_xyz.set_ylabel("position (m)")
        self.ax_xyz.grid(True)
        self.ax_xyz.legend(loc="upper right")

        self.ax_xy.set_title("XY trajectory")
        self.ax_xy.set_xlabel("x (m)")
        self.ax_xy.set_ylabel("y (m)")
        self.ax_xy.axis("equal")
        self.ax_xy.grid(True)
        self.ax_xy.legend(loc="upper right")

        self.last_draw = 0.0

    def add(self, t_rel, x, y, z):
        self.t.append(t_rel)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)

        while self.t and self.t[0] < t_rel - self.window_s:
            self.t.popleft()
            self.x.popleft()
            self.y.popleft()
            self.z.popleft()

        now = time.monotonic()
        if now - self.last_draw < 0.1:
            return
        self.last_draw = now

        ts = list(self.t)
        xs = list(self.x)
        ys = list(self.y)
        zs = list(self.z)

        self.line_x.set_data(ts, xs)
        self.line_y.set_data(ts, ys)
        self.line_z.set_data(ts, zs)
        self.xy_line.set_data(xs, ys)
        self.xy_dot.set_data([xs[-1]], [ys[-1]])

        self.ax_xyz.relim()
        self.ax_xyz.autoscale_view()
        self.ax_xy.relim()
        self.ax_xy.autoscale_view()
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def save_png(self, csv_path):
        png_path = os.path.splitext(csv_path)[0] + ".png"
        self.fig.savefig(png_path, dpi=150)
        return png_path


def default_output_path():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join("logs", f"redis_root_pos_{stamp}.csv")


def main():
    args = parse_args()
    stream_key = args.stream_key or (args.key + ":stream")
    out_path = args.output or default_output_path()
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    stop = False

    def on_sigint(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    r = redis.Redis(host=args.host, port=args.port, db=args.db,
                    socket_timeout=2.0, socket_connect_timeout=2.0)
    r.ping()

    plot = None
    if not args.no_plot:
        try:
            plot = LivePlot(args.window)
        except Exception as exc:
            print(f"[WARN] live plot disabled: {exc}", file=sys.stderr)

    start_id = "0-0" if args.from_start else "$"
    last_id = start_id
    t0_wall = time.time()
    t0_mono = time.monotonic()
    frames = 0
    skipped = 0
    last_print = time.monotonic()
    prev_root = None
    max_step = 0.0

    print(f"[record_redis_root_pos] stream={stream_key} db={args.db} output={out_path}")
    print("[record_redis_root_pos] fields: wall_time,payload_t,root_x,root_y,root_z,stream_id,step_m")

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["wall_time", "rel_time", "payload_t",
                         "root_x", "root_y", "root_z", "stream_id", "step_m"])

        while not stop:
            if args.duration > 0 and time.monotonic() - t0_mono >= args.duration:
                break

            replies = r.xread({stream_key: last_id}, count=32, block=1000)
            if not replies:
                continue

            for _, entries in replies:
                for entry_id, fields in entries:
                    last_id = entry_id.decode() if isinstance(entry_id, bytes) else entry_id
                    blob = fields.get(b"frame")
                    if blob is None:
                        skipped += 1
                        continue

                    try:
                        payload_t, x, y, z = decode_frame(blob, args.dof)
                    except ValueError as exc:
                        skipped += 1
                        print(f"[WARN] skip {last_id}: {exc}", file=sys.stderr)
                        continue

                    wall = time.time()
                    rel = wall - t0_wall
                    if prev_root is None:
                        step = 0.0
                    else:
                        dx = x - prev_root[0]
                        dy = y - prev_root[1]
                        dz = z - prev_root[2]
                        step = (dx * dx + dy * dy + dz * dz) ** 0.5
                    prev_root = (x, y, z)
                    max_step = max(max_step, step)

                    writer.writerow([f"{wall:.6f}", f"{rel:.6f}", f"{payload_t:.6f}",
                                     f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                                     last_id, f"{step:.6f}"])
                    frames += 1

                    if plot is not None:
                        plot.add(rel, x, y, z)

            now = time.monotonic()
            if now - last_print >= 1.0:
                hz = frames / max(1e-6, now - t0_mono)
                print(f"[record_redis_root_pos] frames={frames} avg={hz:.1f}Hz "
                      f"root=({prev_root[0]:.3f},{prev_root[1]:.3f},{prev_root[2]:.3f}) "
                      f"max_step={max_step:.4f}m skipped={skipped} id={last_id}")
                f.flush()
                last_print = now
                max_step = 0.0

    png_path = None
    if plot is not None and frames > 0:
        png_path = plot.save_png(out_path)

    print(f"[record_redis_root_pos] wrote {frames} frames to {out_path}")
    if png_path:
        print(f"[record_redis_root_pos] saved plot to {png_path}")


if __name__ == "__main__":
    main()
