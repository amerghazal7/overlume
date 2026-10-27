#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Convert an ego-robot OBJ mesh (Y up) to a glTF binary (rig frame, Z up) for overlume.

Usage: obj2gltf.py <input.obj> <output.glb>

The OBJ is assumed authored in meters, matching the ROS convention the map/
base_link frames already use -- if the ego mesh renders at the wrong scale,
check units here first, don't add a compensating scale factor blind.
"""
import sys, pathlib
import numpy as np
import trimesh

OBJ_TO_RIG = np.array([[1.0, 0.0,  0.0, 0.0],
                       [0.0, 0.0, -1.0, 0.0],
                       [0.0, 1.0,  0.0, 0.0],
                       [0.0, 0.0,  0.0, 1.0]])

def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__); return 1
    src = pathlib.Path(sys.argv[1])
    dst = pathlib.Path(sys.argv[2])
    mesh = trimesh.load(src, force="scene")
    for geom in mesh.geometry.values():
        geom.apply_transform(OBJ_TO_RIG)
    mesh.export(dst)
    print(f"wrote {dst} ({dst.stat().st_size / 1e6:.1f} MB)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
