#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""flicker_burst_capture.py — VM-077 BEHAVIOR-ribbon live burst capture rig.

Committed replacement for the scratch grab_burst_after.py this investigation
used (the capture logic that actually produced
docs/evidence/vm077-flicker-2026-09-10/rebuild_clip_fix_burst_after_summary.json)
-- fixes two review findings against it. Round 1: any_teal_dropout used an
ABSOLUTE floor (teal_fraction < 0.002), which read a ~13x collapse against
the run's own healthy frames (0.0295 -> 0.0022) as "no dropout" whenever the
floor sat just above the collapsed value. Round 2 (this pass): the round-1
fix's own MEDIAN-relative rule was still wrong the other way -- when the
absent frames are the MAJORITY (the shipped burst: teal_fraction exactly
0.0 on 15/24 frames, ~0.0274 on the other 9), the median itself collapses to
0.0, so `median > 0.0` read a run that is majority-DROPPED as "nothing to
compare against" and reported no dropout. summarize() below instead keys off
the run's own MAX: a frame under half the max, whenever the max clears a
presence floor, is a dropout -- the majority being absent is the strongest
possible dropout signal, never an exemption from detection.

Usage (against an already-configured-and-activated node + bag playback):
    python3 tools/flicker_burst_capture.py --out-dir /tmp/overlume_flicker_burst

Unit-test the detector alone (no ROS): pytest tools/test_flicker_burst_capture.py
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

# ego-front crop: rows 260:560, cols 440:840 -- where LOCAL/BEHAVIOR/velocity
# ribbons nest in the chase-cam framing this rig uses (same crop the repro
# pass's grab_burst.py used, so before/after captures compare pixel-for-pixel).
CROP = (slice(260, 560), slice(440, 840))

IMAGE_TOPIC = "/rendering/image"
N_FRAMES = 24


def corridor_fraction(crop: np.ndarray) -> float:
    """Fraction of a horizontal scan-line through the crop's vertical center
    that reads as ANY ribbon-shaped corridor (r>b -- dark_adas' road/sky are
    not warm-toned; test_ribbon_dropout.cpp's own looks_like_corridor)."""
    row = crop[crop.shape[0] // 2]
    r = row[:, 0].astype(int)
    b = row[:, 2].astype(int)
    return float(np.mean(r > b))


def teal_fraction(crop: np.ndarray) -> float:
    """Fraction of the WHOLE crop that reads as the BEHAVIOR hero ribbon's
    teal/green emissive tint specifically (palette.ribbon_core under bloom):
    G and B both clearly above R."""
    r = crop[:, :, 0].astype(int)
    g = crop[:, :, 1].astype(int)
    b = crop[:, :, 2].astype(int)
    mask = (g > r + 25) & (b > r + 10) & (g > 60)
    return float(np.mean(mask))


def summarize(crops: list[np.ndarray]) -> dict:
    """Per-frame corridor/teal stats plus the run-level dropout flags.

    any_teal_dropout keys off this run's own MAX teal_fraction, not its
    median: a frame below half the max is flagged, gated only by max > 0.0
    (a genuinely all-dark run, where nothing was ever present to collapse
    from, is the only case this excludes -- same "guard the degenerate
    baseline" shape the median version used, just keyed off max instead).
    Median was tried first and is wrong for the exact shape this detector
    exists to catch: when the ribbon is ABSENT on most frames (the shipped
    burst: 15/24 at 0.0, 9/24 at ~0.0274), the median itself is 0.0, so a
    median-gated rule reads "majority dropped" as "nothing to compare
    against" and reports no dropout -- the majority being absent is the
    strongest possible dropout signal, never an exemption. Keying off max
    instead means the healthy frames (however few) still set the bar every
    absent frame is judged against.
    """
    stats = []
    prev = None
    for i, c in enumerate(crops):
        diff = None
        if prev is not None:
            diff = float(np.mean(np.abs(c.astype(int) - prev.astype(int))))
        stats.append({
            "frame": i,
            "corridor_fraction": corridor_fraction(c),
            "teal_fraction": teal_fraction(c),
            "mean_abs_diff_vs_prev": diff,
        })
        prev = c

    teal_fractions = [s["teal_fraction"] for s in stats]
    median_teal = float(np.median(teal_fractions)) if teal_fractions else 0.0
    max_teal = float(np.max(teal_fractions)) if teal_fractions else 0.0
    any_teal_dropout = (
        max_teal > 0.0 and
        any(s["teal_fraction"] < 0.5 * max_teal for s in stats)
    )
    diffs = [s["mean_abs_diff_vs_prev"] for s in stats if s["mean_abs_diff_vs_prev"] is not None]

    return {
        "n_frames": len(crops),
        "any_dropout": any(s["corridor_fraction"] < 0.02 for s in stats),
        "any_teal_dropout": any_teal_dropout,
        "median_teal_fraction": median_teal,
        "min_corridor_fraction": min((s["corridor_fraction"] for s in stats), default=0.0),
        "min_teal_fraction": min(teal_fractions, default=0.0),
        "max_mean_abs_diff": max(diffs) if diffs else None,
        "mean_mean_abs_diff": (sum(diffs) / len(diffs)) if diffs else None,
        "per_frame": stats,
    }


def _run_capture(out_dir: str, deadline_s: float, warmup_s: float) -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from PIL import Image as PILImage

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Node("flicker_burst_capture")
    frames: list[np.ndarray] = []
    node.create_subscription(
        Image, IMAGE_TOPIC,
        lambda m: frames.append(
            np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3).copy()),
        10)

    t0 = time.time()
    deadline = t0 + deadline_s
    while time.time() < deadline and len(frames) < 5000:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > warmup_s and len(frames) >= N_FRAMES:
            break
        if time.time() - t0 <= warmup_s:
            frames.clear()  # discard warmup frames continuously
    burst = frames[:N_FRAMES]
    print(f"captured {len(burst)} frames")

    if burst:
        crops = [f[CROP].copy() for f in burst]
        summary = summarize(crops)

        pil_crops = [PILImage.fromarray(c) for c in crops]
        pil_crops[0].save(os.path.join(out_dir, "burst_after.gif"), save_all=True,
                           append_images=pil_crops[1:], duration=120, loop=0)
        for i in range(min(4, len(burst))):
            PILImage.fromarray(burst[i]).save(os.path.join(out_dir, f"burst_after_full_{i}.png"))

        stack = np.stack([c.astype(np.float32) for c in crops], axis=0)
        stddev = stack.std(axis=0).mean(axis=2)
        heat = (255.0 * (stddev / max(stddev.max(), 1e-6))).astype(np.uint8)
        PILImage.fromarray(heat).save(os.path.join(out_dir, "burst_after_stddev_heatmap.png"))

        with open(os.path.join(out_dir, "burst_after_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        print(json.dumps(summary, indent=2))

    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--deadline-s", type=float, default=240.0)
    ap.add_argument("--warmup-s", type=float, default=45.0,
                     help="seconds of rig warmup to discard before the 24-frame burst")
    args = ap.parse_args()
    _run_capture(args.out_dir, args.deadline_s, args.warmup_s)


if __name__ == "__main__":
    main()
