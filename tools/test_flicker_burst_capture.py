# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Unit test for the relative-median teal-dropout detector (no ROS needed).

Run: pytest tools/test_flicker_burst_capture.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from flicker_burst_capture import summarize

def _solid_crop(r: int, g: int, b: int, size: int = 8) -> np.ndarray:
    crop = np.zeros((size, size, 3), dtype=np.uint8)
    crop[:, :, 0] = r
    crop[:, :, 1] = g
    crop[:, :, 2] = b
    return crop

def test_flags_relative_collapse_the_absolute_floor_missed():
    healthy = _solid_crop(50, 120, 100)
    collapsed = _solid_crop(50, 65, 60)
    crops = [collapsed] + [healthy] * 15 + [collapsed] * 7 + [healthy]
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is True

def test_no_dropout_on_a_healthy_run():
    healthy = _solid_crop(50, 120, 100)
    crops = [healthy] * 24
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is False

def test_all_dark_run_does_not_false_positive():
    dark = _solid_crop(10, 10, 10)
    crops = [dark] * 24
    summary = summarize(crops)
    assert summary["any_teal_dropout"] is False
    assert summary["median_teal_fraction"] == 0.0

def test_flags_majority_absent_minority_present_dropout():
    healthy = _solid_crop(50, 120, 100)
    absent = _solid_crop(50, 65, 60)
    crops = [absent] * 15 + [healthy] * 9
    summary = summarize(crops)
    assert summary["median_teal_fraction"] == 0.0
    assert summary["any_teal_dropout"] is True
