#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""flicker_capture.py — VM-077 carpet-flicker frame-diff rig (2026-09-10).

Committed replacement for the un-recoverable `flicker_capture.py` the
2026-09-10 code review found missing (the N=24 round's raw per-condition
data survived only because its OUTPUT json/pngs happened to still be on
disk in /tmp — the capture script itself did not). This script fixes both
review findings at once:

  1. it exists and is committed, so the NEXT measurement pass leaves a
     reproducible artifact on disk / in git, not just in /tmp;
  2. it fixes the N=24 round's own confound (carpet ON and OFF were two
     SEPARATE `ros2 bag play` launches, landing at different points in the
     looped bag -- carpet_msg_rate_hz measured 9.28 Hz ON vs 5.35 Hz OFF,
     i.e. not the same window) by capturing BOTH conditions from a SINGLE
     continuous node + bag-play session, toggling the already-live
     `layer_trajectory_carpet` ros2 param back and forth. `apply_layer_gates()`
     (scene_assembly.cpp) only clears the per-tick SceneAssembly vector before
     the renderer sees it -- the adapter's ingest/rebuild bookkeeping in
     trajectory_carpet.cpp runs identically either way -- so this toggle is a
     pure render-visibility gate, not a different code path, and both
     conditions observe the same underlying `output_trajectory_carpet`
     message stream.

Usage (against an already-configured-and-activated node + bag playback --
see flicker_measure.sh, which drives this):
    python3 tools/flicker_capture.py --out-dir /tmp/overlume_flicker_measure_after

Frame diff metric: mean absolute per-pixel delta (across RGB channels) in a
fixed ego-centered crop, between consecutive frames of the SAME condition
within one contiguous capture block (never across a block boundary, which
would straddle the other condition's block and the param-toggle settle
time). Output schema matches the preserved pre-fix
docs/evidence/vm077-flicker-2026-09-10/carpet_{on,off}_summary.json exactly,
so the two rounds compare field-for-field.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import time

import numpy as np

# Ego-centered crop (fraction of width/height): a chase-cam framing puts the
# ego roughly center-lower. ponytail: fixed fractions, not detected from the
# scene -- good enough for a relative ON-vs-OFF comparison; revisit only if a
# future rig needs the exact crop the (unrecoverable) original script used.
CROP_X = (0.30, 0.70)
CROP_Y = (0.30, 0.90)

NODE_NAME = "/overlume_node"
IMAGE_TOPIC = "/rendering/image"
CARPET_TOPIC = "/navigation_motion_obstacle_planner_node/output_trajectory_carpet"


def set_layer_param(value: bool) -> None:
    subprocess.run(
        ["ros2", "param", "set", NODE_NAME, "layer_trajectory_carpet", "true" if value else "false"],
        check=True, capture_output=True, text=True,
    )


def crop(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    x0, x1 = int(w * CROP_X[0]), int(w * CROP_X[1])
    y0, y1 = int(h * CROP_Y[0]), int(h * CROP_Y[1])
    return frame[y0:y1, x0:x1]


class Capture:
    def __init__(self, node, image_topic: str, carpet_topic: str):
        from sensor_msgs.msg import Image
        from visualization_msgs.msg import MarkerArray
        import rclpy.qos as qos

        self.frames: list[tuple[float, np.ndarray]] = []
        self.carpet_stamps: list[float] = []
        self._recording = False

        node.create_subscription(
            Image, image_topic, self._on_image,
            qos.QoSProfile(depth=5, reliability=qos.ReliabilityPolicy.RELIABLE),
        )
        node.create_subscription(
            MarkerArray, carpet_topic, self._on_carpet,
            qos.QoSProfile(depth=10, reliability=qos.ReliabilityPolicy.RELIABLE),
        )

    def _on_image(self, msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if self._recording:
            self.frames.append((t, arr.copy()))

    def _on_carpet(self, msg) -> None:
        if msg.markers:
            t = msg.markers[0].header.stamp.sec + msg.markers[0].header.stamp.nanosec * 1e-9
        else:
            t = time.time()
        self.carpet_stamps.append(t)

    def record_block(self, n_frames: int, executor) -> list[tuple[float, np.ndarray]]:
        """Spin until n_frames NEW frames have arrived; return just those."""
        start_len = len(self.frames)
        self._recording = True
        while len(self.frames) - start_len < n_frames:
            executor.spin_once(timeout_sec=0.5)
        self._recording = False
        return self.frames[start_len:start_len + n_frames]


def summarize(block_groups: list[list[tuple[float, np.ndarray]]], carpet_stamps: list[float]) -> tuple[dict, np.ndarray, np.ndarray]:
    """block_groups: list of contiguous same-condition frame blocks (each a
    list of (t, frame)). Diffs/dt computed WITHIN each block only, then
    concatenated -- never across a block boundary (that gap includes the
    other condition's block + the param-toggle settle sleep)."""
    all_diffs: list[float] = []
    all_dts: list[float] = []
    all_marks: list[int] = []
    all_frames: list[np.ndarray] = []
    wall_span = 0.0
    n_frames_total = 0
    carpet_msgs_in_window = 0

    for block in block_groups:
        n_frames_total += len(block)
        wall_span += block[-1][0] - block[0][0]
        all_frames.extend(f for _, f in block)
        for (t0, f0), (t1, f1) in zip(block, block[1:]):
            dt = t1 - t0
            c0 = crop(f0).astype(np.float32)
            c1 = crop(f1).astype(np.float32)
            all_diffs.append(float(np.mean(np.abs(c1 - c0))))
            all_dts.append(dt)
            all_marks.append(1 if any(t0 <= s < t1 for s in carpet_stamps) else 0)
        block_lo, block_hi = block[0][0], block[-1][0]
        carpet_msgs_in_window += sum(1 for s in carpet_stamps if block_lo <= s <= block_hi)

    mean_dt = sum(all_dts) / len(all_dts) if all_dts else 0.0
    summary = {
        "n_frames": n_frames_total,
        "wall_span_s": wall_span,
        "mean_frame_dt_s": mean_dt,
        "approx_fps": (1.0 / mean_dt) if mean_dt > 0 else 0.0,
        "frame_diffs": all_diffs,
        "frame_dt_s": all_dts,
        "carpet_msgs_in_window": carpet_msgs_in_window,
        "carpet_msg_rate_hz": (carpet_msgs_in_window / wall_span) if wall_span > 0 else 0.0,
        "carpet_msg_in_interval": all_marks,
    }
    stack = np.stack(all_frames).astype(np.float32)
    stddev = stack.std(axis=0).mean(axis=-1)  # per-pixel stddev, channel-averaged
    return summary, stddev, all_frames[0]


def save_heatmap(stddev: np.ndarray, path: str) -> None:
    from PIL import Image as PILImage

    lo, hi = stddev.min(), stddev.max()
    norm = np.zeros_like(stddev, dtype=np.uint8) if hi <= lo else \
        ((stddev - lo) / (hi - lo) * 255).astype(np.uint8)
    PILImage.fromarray(norm, mode="L").resize((320, 240)).save(path)


def save_frame(frame: np.ndarray, path: str) -> None:
    from PIL import Image as PILImage

    PILImage.fromarray(frame, mode="RGB").save(path)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--block-frames", type=int, default=12, help="frames captured per ON/OFF block")
    ap.add_argument("--rounds", type=int, default=2, help="number of ON+OFF block pairs")
    ap.add_argument("--settle-s", type=float, default=0.3, help="sleep after each param toggle")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = rclpy.create_node("flicker_capture")
    cap = Capture(node, IMAGE_TOPIC, CARPET_TOPIC)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Drain a bit of backlog before starting so the first recorded block
    # isn't the node's very first (potentially unsteady) frames.
    for _ in range(20):
        executor.spin_once(timeout_sec=0.5)

    on_blocks: list[list[tuple[float, np.ndarray]]] = []
    off_blocks: list[list[tuple[float, np.ndarray]]] = []
    for round_i in range(args.rounds):
        set_layer_param(True)
        time.sleep(args.settle_s)
        on_blocks.append(cap.record_block(args.block_frames, executor))
        print(f"[round {round_i}] ON block: {len(on_blocks[-1])} frames")

        set_layer_param(False)
        time.sleep(args.settle_s)
        off_blocks.append(cap.record_block(args.block_frames, executor))
        print(f"[round {round_i}] OFF block: {len(off_blocks[-1])} frames")

    # Leave the layer enabled (the node's own default) on exit.
    set_layer_param(True)

    on_summary, on_stddev, on_f0 = summarize(on_blocks, cap.carpet_stamps)
    off_summary, off_stddev, off_f0 = summarize(off_blocks, cap.carpet_stamps)

    with open(os.path.join(args.out_dir, "carpet_on_summary.json"), "w") as f:
        json.dump(on_summary, f, indent=2)
    with open(os.path.join(args.out_dir, "carpet_off_summary.json"), "w") as f:
        json.dump(off_summary, f, indent=2)
    save_heatmap(on_stddev, os.path.join(args.out_dir, "carpet_on_stddev_heatmap.png"))
    save_heatmap(off_stddev, os.path.join(args.out_dir, "carpet_off_stddev_heatmap.png"))
    save_frame(on_f0, os.path.join(args.out_dir, "carpet_on_f0.png"))
    save_frame(off_f0, os.path.join(args.out_dir, "carpet_off_f0.png"))

    print(json.dumps({
        "on": {k: v for k, v in on_summary.items() if k not in ("frame_diffs", "frame_dt_s", "carpet_msg_in_interval")},
        "off": {k: v for k, v in off_summary.items() if k not in ("frame_diffs", "frame_dt_s", "carpet_msg_in_interval")},
    }, indent=2))

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
