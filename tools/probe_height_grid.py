#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""probe_height_grid.py — checks the /debug_ogm_2 metric height encoding against /debug_ogm_1.

/debug_ogm_1 is the geometric node's height map min-max normalised per frame into 1..100 (int8 -1 =
unknown); /debug_ogm_2 is the same height map in a FIXED metric window (0..100 over
[--min-m, --max-m] metres above the grid origin plane, -1 = unknown). For the same lidar frame the
two must agree on which cells are unknown and be linearly related everywhere the metric encoding is
not clamped.

Live mode (needs a sourced ROS 2 environment and the perception costmap nodes running):
    python3 tools/probe_height_grid.py --seconds 10
subscribes best-effort to both topics, pairs frames by header stamp and prints PASS/FAIL for
  * /debug_ogm_2 arrival rate >= 5 Hz,
  * identical unknown masks on every paired frame,
  * at least 10 qualifying frames, and
  * Pearson r > 0.99 between /debug_ogm_1 values and decoded /debug_ogm_2 heights on every
    qualifying frame (worst frame reported).
A frame qualifies when >= 100 cells are known in both topics and not clamped in /debug_ogm_2 AND
those cells span >= 5 distinct /debug_ogm_2 levels. Other frames (all-unknown right after a node
start, near-flat ground where the fixed 0.05 m quantisation leaves one or two levels) cannot yield
a meaningful r; they are reported on an INFO line and excluded, never failed on r.
Exit code 0 only when every PASS/FAIL line passes. The checks are scale-invariant, so a
--min-m/--max-m that differs from the perception config still passes; a window mismatch only shows
as wrong terrain heights by eye.

    python3 tools/probe_height_grid.py --self-test
runs the same comparison on synthetic round trips (no ROS traffic) and checks that deliberately
broken encodings fail and that excluded frames do not.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import sys
import time
from types import SimpleNamespace

import numpy as np

OGM1_TOPIC = "/debug_ogm_1"
OGM2_TOPIC = "/debug_ogm_2"
MIN_RATE_HZ = 5.0
MIN_PEARSON_R = 0.99
MIN_USED_CELLS = 100   # cells known in both and unclamped in ogm2 for a frame to qualify
MIN_LEVELS = 5         # distinct unclamped ogm2 levels for a frame to qualify
MIN_QUALIFYING = 10    # qualifying frames needed for the run to pass
UNKNOWN = -1
LEVELS = 100
MAX_PENDING = 32  # unpaired frames kept per topic while waiting for the other one


def decode_linear(values_int8, min_m, max_m):
    """Decode /debug_ogm_2 cells to metres: min + v/100*(max-min); NaN for -1 and any invalid value."""
    v = np.asarray(values_int8, dtype=np.int8)
    out = np.full(v.shape, np.nan, dtype=np.float64)
    ok = (v >= 0) & (v <= LEVELS)
    out[ok] = min_m + v[ok].astype(np.float64) / LEVELS * (max_m - min_m)
    return out


def compare(ogm1_int8, ogm2_int8, min_m, max_m):
    """Compare one /debug_ogm_1 frame with its /debug_ogm_2 partner.

    Returns unknown_mask_equal (both topics mark the same cells -1), pearson_r over cells known in
    both and not clamped in ogm2 (level 0 or 100), n_known (known in both), n_used (the cells r was
    computed over) and n_levels (distinct ogm2 levels among those cells). pearson_r is NaN when
    fewer than 2 cells remain or either side is constant.
    """
    a = np.asarray(ogm1_int8, dtype=np.int8)
    b = np.asarray(ogm2_int8, dtype=np.int8)
    if a.shape != b.shape:
        raise ValueError(f"shape mismatch: {a.shape} vs {b.shape}")
    unk1 = a == UNKNOWN
    unk2 = b == UNKNOWN
    heights2 = decode_linear(b, min_m, max_m)
    used = ~unk1 & ~unk2 & (b > 0) & (b < LEVELS)
    r = float("nan")
    if int(used.sum()) >= 2:
        x = a[used].astype(np.float64)
        y = heights2[used]
        if x.std() > 0.0 and y.std() > 0.0:
            r = float(np.corrcoef(x, y)[0, 1])
    return {
        "unknown_mask_equal": bool(np.array_equal(unk1, unk2)),
        "pearson_r": r,
        "n_known": int((~unk1 & ~unk2).sum()),
        "n_used": int(used.sum()),
        "n_levels": int(np.unique(b[used]).size),
    }


def qualifies(result):
    """A frame is judged on r only with enough unclamped cells spanning enough distinct levels."""
    return bool(result["n_used"] >= MIN_USED_CELLS and result["n_levels"] >= MIN_LEVELS)


def passes(result):
    """One qualifying frame passes when the unknown masks agree and r > MIN_PEARSON_R (NaN fails)."""
    return bool(result["unknown_mask_equal"] and result["pearson_r"] > MIN_PEARSON_R)


# --- live pairing and report ----------------------------------------------------------------


def _line(ok, text):
    print(f"{'PASS' if ok else 'FAIL'}  {text}")
    return ok


class Pairer:
    """Collects both topics, pairs frames by exact header stamp and compares each pair."""

    def __init__(self, min_m, max_m):
        self.min_m = min_m
        self.max_m = max_m
        self.pending = {1: {}, 2: {}}
        self.count = {1: 0, 2: 0}
        self.arrivals2 = []
        self.results = []
        self.malformed = 0
        self.shape_mismatch = 0

    def on_msg(self, which, msg):
        now = time.monotonic()
        self.count[which] += 1
        if which == 2:
            self.arrivals2.append(now)
        grid = np.asarray(msg.data, dtype=np.int8)
        if grid.size == 0 or grid.size != msg.info.width * msg.info.height:
            self.malformed += 1
            return
        grid = grid.reshape(msg.info.height, msg.info.width)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        other = self.pending[3 - which].pop(key, None)
        if other is None:
            mine = self.pending[which]
            mine[key] = grid
            while len(mine) > MAX_PENDING:
                mine.pop(next(iter(mine)))  # dicts keep insertion order: drop the oldest
            return
        a, b = (grid, other) if which == 1 else (other, grid)
        if a.shape != b.shape:
            self.shape_mismatch += 1
            return
        self.results.append(compare(a, b, self.min_m, self.max_m))

    def rate2_hz(self):
        if len(self.arrivals2) < 2:
            return 0.0
        span = self.arrivals2[-1] - self.arrivals2[0]
        return (len(self.arrivals2) - 1) / span if span > 0.0 else 0.0


def report(p, check_rate=True):
    """Print the verdict lines for a Pairer; True only when every PASS/FAIL line passes."""
    ok = True
    if check_rate:
        rate = p.rate2_hz()
        ok &= _line(rate >= MIN_RATE_HZ,
                    f"{OGM2_TOPIC} rate  ({rate:.1f} Hz, need >= {MIN_RATE_HZ:.1f}; {p.count[2]} msgs, "
                    f"{OGM1_TOPIC} {p.count[1]} msgs)")
    n = len(p.results)
    if n == 0:
        _line(False, f"stamp-paired frames  (0 pairs; {p.malformed} malformed, "
                     f"{p.shape_mismatch} shape mismatches)")
        return False
    ok &= _line(True, f"stamp-paired frames  ({n} pairs)")
    equal = sum(1 for r in p.results if r["unknown_mask_equal"])
    ok &= _line(equal == n, f"unknown masks equal  ({equal}/{n} pairs)")

    qual = [r for r in p.results if qualifies(r)]
    n_sparse = sum(1 for r in p.results if r["n_used"] < MIN_USED_CELLS)
    n_flat = n - len(qual) - n_sparse
    print(f"INFO  excluded for insufficient relief  ({n - len(qual)}/{n} frames: {n_sparse} with "
          f"< {MIN_USED_CELLS} usable cells, {n_flat} with < {MIN_LEVELS} distinct levels)")
    ok &= _line(len(qual) >= MIN_QUALIFYING,
                f"qualifying frames  ({len(qual)}, need >= {MIN_QUALIFYING})")
    if qual:
        rs = [r["pearson_r"] for r in qual]
        worst = float("nan") if any(np.isnan(rs)) else min(rs)
        used = int(np.median([r["n_used"] for r in qual]))
        ok &= _line(worst > MIN_PEARSON_R,
                    f"pearson r  (worst of {len(qual)} qualifying {worst:.4f}, need > {MIN_PEARSON_R}; "
                    f"median {used} cells/frame)")
    return bool(ok)


def run_live(min_m, max_m, seconds):
    try:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        print(f"FAIL  rclpy not importable ({exc}); source the ROS 2 environment first")
        return 1

    rclpy.init()
    node = rclpy.create_node("probe_height_grid")
    pairer = Pairer(min_m, max_m)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(OccupancyGrid, OGM1_TOPIC, lambda m: pairer.on_msg(1, m), qos)
    node.create_subscription(OccupancyGrid, OGM2_TOPIC, lambda m: pairer.on_msg(2, m), qos)

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    return 0 if report(pairer) else 1


# --- self-test -------------------------------------------------------------------------------


def _synthetic_field(n=120, seed=7):
    """Slope + berm + ditch + noise, an unknown patch and two out-of-window patches (metres)."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:n, 0:n].astype(np.float64)
    h = 0.015 * (x - n / 2.0)
    h += 1.2 * np.exp(-((x - 30) ** 2 + (y - 40) ** 2) / (2 * 6.0 ** 2))
    h -= 1.0 * np.exp(-((x - 80) ** 2 + (y - 70) ** 2) / (2 * 8.0 ** 2))
    h += rng.normal(0.0, 0.02, h.shape)
    h[100:104, 10:14] = 4.0   # above the +3 m window: clamps to level 100
    h[5:8, 100:103] = -3.0    # below the -2 m window: clamps to level 0
    h[50:60, 90:100] = np.nan  # unknown patch
    return h


def _encode_per_frame(h):
    """Mirror of the node's /debug_ogm_1 encoding: known cells min-max scaled to 1..100, else 255."""
    known = ~np.isnan(h)
    lo, hi = float(h[known].min()), float(h[known].max())
    v = (np.where(known, h, lo) - lo) * (99.0 / max(1e-6, hi - lo)) + 1.0
    v = np.where(known, np.floor(v + 0.5), 255.0)
    return v.astype(np.uint8).view(np.int8)  # the uint8 255 goes on the wire as int8 -1


def _encode_metric(h, min_m, max_m):
    """Mirror of encode_height_cell (perception_grid_map/height_encoding.h); unknown -> -1."""
    known = ~np.isnan(h)
    x = (np.where(known, h, min_m) - min_m) / (max_m - min_m) * LEVELS
    v = np.clip(np.floor(x + 0.5), 0, LEVELS)
    return np.where(known, v, UNKNOWN).astype(np.int8)


def _msg(stamp_sec, grid):
    """Minimal stand-in for nav_msgs/OccupancyGrid as far as Pairer.on_msg reads it."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=0)),
        info=SimpleNamespace(width=grid.shape[1], height=grid.shape[0]),
        data=grid.reshape(-1).tolist())


def _run_pairer(frames, min_m, max_m):
    """Feed (ogm1, ogm2) frame pairs through a Pairer; returns (report ok, printed text)."""
    p = Pairer(min_m, max_m)
    for k, (a, b) in enumerate(frames):
        p.on_msg(1, _msg(k, a))
        p.on_msg(2, _msg(k, b))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ok = report(p, check_rate=False)  # synthetic feed has no meaningful arrival rate
    return ok, buf.getvalue()


def self_test():
    min_m, max_m = -2.0, 3.0

    d = decode_linear(np.array([-1, 0, 50, 100, 101, -5], dtype=np.int8), min_m, max_m)
    assert np.isnan(d[0]) and np.isnan(d[4]) and np.isnan(d[5]), d
    assert d[1] == -2.0 and d[2] == 0.5 and d[3] == 3.0, d

    h = _synthetic_field()
    o1 = _encode_per_frame(h)
    o2 = _encode_metric(h, min_m, max_m)

    good = compare(o1, o2, min_m, max_m)
    assert good["unknown_mask_equal"], good
    assert good["pearson_r"] > MIN_PEARSON_R, good
    assert good["n_known"] == int((~np.isnan(h)).sum()), good
    assert good["n_used"] < good["n_known"], good  # the clamped patches were excluded
    assert qualifies(good) and passes(good), good

    # One cell known in ogm1 but unknown in ogm2: the masks differ.
    one_off = o2.copy()
    one_off[tuple(np.argwhere(one_off != UNKNOWN)[0])] = UNKNOWN
    bad_mask = compare(o1, one_off, min_m, max_m)
    assert not bad_mask["unknown_mask_equal"] and not passes(bad_mask), bad_mask

    # Same known set but the values scrambled: masks equal, r near 0, still a qualifying frame.
    shuffled = o2.copy()
    idx = np.flatnonzero(shuffled != UNKNOWN)
    shuffled.flat[idx] = shuffled.flat[np.random.default_rng(1).permutation(idx)]
    bad_r = compare(o1, shuffled, min_m, max_m)
    assert bad_r["unknown_mask_equal"] and abs(bad_r["pearson_r"]) < 0.5, bad_r
    assert qualifies(bad_r) and not passes(bad_r), bad_r

    # Constant ogm2 (a dead encoder): one level, so the frame is excluded, not scored.
    dead = np.where(o2 != UNKNOWN, 50, UNKNOWN).astype(np.int8)
    bad_dead = compare(o1, dead, min_m, max_m)
    assert np.isnan(bad_dead["pearson_r"]) and not qualifies(bad_dead), bad_dead

    # Excluded-not-failed (a): an all-unknown pair, as right after the costmap node starts.
    unk = np.full(h.shape, UNKNOWN, dtype=np.int8)
    all_unknown = compare(unk, unk, min_m, max_m)
    assert all_unknown["unknown_mask_equal"] and all_unknown["n_used"] == 0, all_unknown
    assert np.isnan(all_unknown["pearson_r"]) and not qualifies(all_unknown), all_unknown

    # Excluded-not-failed (b): near-flat ground, 0.01 m noise on every known cell. ogm2 spans only a
    # couple of 0.05 m levels while ogm1 stretches the same noise over 1..100.
    flat_h = np.random.default_rng(3).normal(0.0, 0.01, h.shape)
    flat1 = _encode_per_frame(flat_h)
    flat2 = _encode_metric(flat_h, min_m, max_m)
    flat = compare(flat1, flat2, min_m, max_m)
    assert flat["unknown_mask_equal"] and flat["n_used"] >= MIN_USED_CELLS, flat
    assert flat["n_levels"] < MIN_LEVELS and not qualifies(flat), flat

    # Run-level verdicts through the real Pairer + report path (excluded frames never fail on r).
    goods = []
    for seed in range(12):
        hs = _synthetic_field(seed=seed)
        goods.append((_encode_per_frame(hs), _encode_metric(hs, min_m, max_m)))
    excluded = [(unk, unk), (flat1, flat2)]

    ok, text = _run_pairer(goods + excluded, min_m, max_m)
    assert ok, text                                  # 12 good + 2 excluded: PASS
    assert "excluded for insufficient relief  (2/14" in text, text

    ok, text = _run_pairer(goods[:9] + excluded, min_m, max_m)
    assert not ok and "FAIL  qualifying frames  (9" in text, text   # too few qualifying frames

    ok, text = _run_pairer([(o1, dead)] * 12, min_m, max_m)
    assert not ok and "FAIL  qualifying frames  (0" in text, text   # dead encoder cannot pass

    ok, text = _run_pairer(goods + [(unk, flat2)], min_m, max_m)
    assert not ok and "FAIL  unknown masks equal" in text, text     # masks are checked on every pair

    ok, text = _run_pairer(goods[:11] + [(o1, shuffled)], min_m, max_m)
    assert not ok and "FAIL  pearson r" in text, text               # one bad qualifying frame fails

    print(f"PASS  self-test  (r={good['pearson_r']:.5f}, n_used={good['n_used']}/{good['n_known']}, "
          f"3 broken cases rejected, 2 low-relief cases excluded, 5 run verdicts)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check /debug_ogm_2 metric heights against /debug_ogm_1.")
    ap.add_argument("--self-test", action="store_true", help="synthetic round trip, no ROS")
    ap.add_argument("--min-m", type=float, default=-2.0, help="encoding window floor [m]")
    ap.add_argument("--max-m", type=float, default=3.0, help="encoding window ceiling [m]")
    ap.add_argument("--seconds", type=float, default=10.0, help="live capture duration")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.max_m <= args.min_m:
        ap.error("--max-m must be greater than --min-m")
    return run_live(args.min_m, args.max_m, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
