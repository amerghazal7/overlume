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

# Registration: the shipped pointcloud_transform must put the fixture bag's ground at the bowl floor
# (rig z ~ 0). It was 1.15 (the m2o1 value) vs the bag's 2.4 lidar height: ground 1.28 m too low,
# crosswalk stripes doubled. Optional argv[2] = bag dir; skipped when bag or ROS is absent.
import os
bag = os.path.expanduser(sys.argv[2]) if len(sys.argv) > 2 else ""
db3 = os.path.join(bag, "stack_v3_full_sensors_2026-09-11_0.db3")
try:
    import sqlite3

    import numpy as np
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2
    assert os.path.exists(db3)
except (ImportError, AssertionError):
    print("SKIP hybrid registration (no bag/ROS)")
    sys.exit(0)
db = sqlite3.connect(db3)
tid = db.execute("select id from topics where name=?", (p["pointcloud_topic"],)).fetchone()[0]
row = db.execute("select data from messages where topic_id=? limit 1 offset 300", (tid,)).fetchone()
P = np.array([[a, b, c] for a, b, c, *_ in pc2.read_points(
    deserialize_message(row[0], PointCloud2), field_names=("x", "y", "z"), skip_nans=True)])
t = p["pointcloud_transform"]
z = P @ np.array(t[6:9]) + t[11]  # z row of R is t[6:9]: layout is R row-major then t
near = z[(np.hypot(P[:, 0], P[:, 1]) < 12) & (z < 1.0)]
ground = np.median(near[near < np.percentile(near, 30)])
assert abs(ground) < 0.15, f"hybrid lidar ground at rig z={ground:.2f} m (want ~0): wrong lidar height"
print("PASS hybrid lidar registration, ground z=%.3f" % ground)
