#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
"""Shipped hybrid must name a cloud topic (else it silently renders bowl only)."""
import sys

import yaml

p = yaml.safe_load(open(sys.argv[1]))["/**"]["ros__parameters"]
assert not p["hybrid_enabled"] or p["pointcloud_topic"], \
    "hybrid_enabled with empty pointcloud_topic"
assert p["splat_radius"] == 3, p["splat_radius"]
print("PASS default_params hybrid")
