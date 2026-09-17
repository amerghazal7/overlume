#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Generate `camera_extrinsics` (camera->rig [R|t]) from the CARLA sim config.

The live cameras are published by the CARLA bridge, so the ground-truth mounting
is in micropilot_sim/config/carla_interface_config.yaml. This converts each
camera's CARLA spawn_point into the rig-frame [R(9 row-major) | t(3)] layout the
rendering node expects.

Frames:
  CARLA  : left-handed, x-forward, y-right, z-up, angles in degrees, origin ~ground.
  Rig    : right-handed, x-forward, y-left, z-up, ground at z=0 (ROS REP-103).
Conversion (matches carla_ros_bridge): position (x,-y,z); actor orientation via
RPY(roll, -pitch, -yaw); camera optical axes in the body frame are
  right = -body_y, down = -body_z, fwd = +body_x  (CV optical convention).

Usage:
  python3 extrinsics_from_carla.py [carla_interface_config.yaml] \
      [--order fl_camera,fm_camera,fr_camera,bl_camera,bm_camera,br_camera]
Prints the YAML `camera_extrinsics:` block to stdout.
"""
import argparse
import os
import numpy as np
import yaml

# Default comes from the environment (this is a deployment-specific file); pass
# --cfg explicitly otherwise.
DEF_CFG = os.environ.get("CARLA_INTERFACE_CONFIG", "carla_interface_config.yaml")
DEF_ORDER = "fl_camera,fm_camera,fr_camera,bl_camera,bm_camera,br_camera"


def _rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def _ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def _rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

# camera optical axes (right, down, fwd) expressed in the ROS body frame
R_BODY_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


def extrinsics(cfg_path, order):
    cams = {c["name"]: c for c in yaml.safe_load(open(cfg_path))["cameras"]}
    out = []
    for nm in order:
        sp = cams[nm]["spawn_point"]
        t = np.array([sp["x"], -sp["y"], sp["z"]])
        roll, pitch, yaw = np.radians([sp["roll"], -sp["pitch"], -sp["yaw"]])
        R = (_rz(yaw) @ _ry(pitch) @ _rx(roll)) @ R_BODY_OPT
        out.append((nm, R, t))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cfg", nargs="?", default=DEF_CFG)
    ap.add_argument("--order", default=DEF_ORDER)
    a = ap.parse_args()
    rows = extrinsics(a.cfg, a.order.split(","))
    print("    camera_extrinsics:")
    for nm, R, t in rows:
        print(f"      # {nm}")
        for v in list(R.reshape(-1)) + list(t):
            print(f"      - {float(v):.10g}")


if __name__ == "__main__":
    main()
