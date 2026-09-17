# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Unit test for the relative-median teal-dropout detector (no ROS needed).

Run: pytest tools/test_flicker_burst_capture.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from flicker_burst_capture import summarize  # noqa: E402


def _solid_crop(r: int, g: int, b: int, size: int = 8) -> np.ndarray:
    crop = np.zeros((size, size, 3), dtype=np.uint8)
    crop[:, :, 0] = r
    crop[:, :, 1] = g
    crop[:, :, 2] = b
    return crop


def test_flags_relative_collapse_the_absolute_floor_missed():
    # Reproduces the exact shape of the miscalibrated capture: 16 healthy
    # teal frames plus 8 collapsed frames whose teal_fraction is still above
    # the old absolute floor (0.002) but ~13x below the run's own median --
    # the case the old detector read as "no dropout".
    healthy = _solid_crop(50, 120, 100)      # teal_fraction == 1.0
    collapsed = _solid_crop(50, 65, 60)      # g/b margins fail -> teal_fraction == 0.0
    crops = [collapsed] + [healthy] * 15 + [collapsed] * 7 + [healthy]
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is True


def test_no_dropout_on_a_healthy_run():
    healthy = _solid_crop(50, 120, 100)
    crops = [healthy] * 24
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is False


def test_all_dark_run_does_not_false_positive():
    # median teal_fraction == 0.0 here -- nothing to "collapse" relative to.
    dark = _solid_crop(10, 10, 10)
    crops = [dark] * 24
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is False
    assert summary["median_teal_fraction"] == 0.0


def test_flags_majority_absent_minority_present_dropout():
    # Pins the exact shape of the shipped rebuild_clip_fix_burst_after_summary.json:
    # 15/24 frames at teal_fraction == 0.0, 9/24 at a healthy fraction. The
    # median-gated detector (round 1's fix) read this as "no dropout" because
    # the ABSENT majority pulls the median itself down to 0.0 -- a majority
    # dropout is the strongest possible signal, not an exemption.
    healthy = _solid_crop(50, 120, 100)
    absent = _solid_crop(50, 65, 60)
    crops = [absent] * 15 + [healthy] * 9
    summary = summarize(crops)
    assert summary["median_teal_fraction"] == 0.0
    assert summary["any_teal_dropout"] is True
