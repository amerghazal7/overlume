#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
"""Normalize a source glTF/GLB/OBJ model into overlume's per-class
clay-object convention (Epic 2 Task 4 / VM-022).

Usage: normalize_models.py <input.glb> [output.glb]
       normalize_models.py --selfcheck

(output defaults to the input path with its extension swapped to .glb)

What "normalized" means here, precisely:
  - Source models are assumed authored glTF-style Y-up (Kenney/Quaternius
    packs both are). Rotated into this renderer's ROS convention,
    +X-forward / +Z-up, via the axis permutation (x,y,z) -> (z,x,y) -- a
    proper rotation (determinant +1: a 120-degree turn about the (1,1,1)
    axis), so winding order/normals need no flip.
  - Recentered on its OWN footprint centre in the new X/Y (forward/width)
    plane, and dropped so its lowest point sits at Z=0 -- the same
    ground-contact-origin convention ego.cpp's build_ego_box() already
    uses ("origin is ground-contact point").
  - Scaled to a UNIT footprint: after centering, X and Y extents are each
    independently rescaled to exactly 1.0 (non-uniform -- a real object's
    footprint is rarely square), so a caller can later derive a per-object
    transform as `scale = TrackedObject.dimensions / (1, 1, own_height)`
    (renderer.cpp / objects.cpp). Height (Z) is rescaled by the SAME factor
    as X (the forward/length axis) -- ponytail: an arbitrary but documented
    choice between "scale by X" vs "scale by Y" vs "geometric mean"; all
    three are equally defensible for a cosmetic model swap and only one
    was needed. Upgrade if a shipped model ever looks visibly squashed.

Self-check (--selfcheck): builds a generated unit cube, normalizes it, and
asserts the output's X/Y bounds are [-0.5, 0.5] -- the smallest thing that
fails if the recenter/scale math breaks.
"""
import sys
import pathlib

import numpy as np
import trimesh

# (x, y, z) -> (z, x, y): proper rotation (det +1), Y-up source -> +X-forward/
# +Z-up target.
_ROTATE_YUP_TO_ROS = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def normalize_scene(scene: trimesh.Scene) -> trimesh.Scene:
    """Applies the rotate/recenter/scale pipeline described in this module's
    docstring to `scene` in place (via apply_transform, so multi-material
    node hierarchy is preserved -- see obj2gltf_m02p.py's own comment on
    force="scene" for why that matters), and returns it."""
    scene.apply_transform(_ROTATE_YUP_TO_ROS)

    bounds = scene.bounds  # already in the rotated (ROS) frame
    mins, maxs = bounds[0], bounds[1]
    center_x = (mins[0] + maxs[0]) / 2.0
    center_y = (mins[1] + maxs[1]) / 2.0
    ground_z = mins[2]
    scene.apply_translation([-center_x, -center_y, -ground_z])

    extent_x = maxs[0] - mins[0]
    extent_y = maxs[1] - mins[1]
    if extent_x <= 0.0 or extent_y <= 0.0:
        raise ValueError(f"degenerate footprint (extent_x={extent_x}, extent_y={extent_y})")
    scale_x = 1.0 / extent_x
    scale_y = 1.0 / extent_y
    scale_forward = scale_x  # see docstring: height rescaled by the X factor
    scale_matrix = np.diag([scale_x, scale_y, scale_forward, 1.0])
    scene.apply_transform(scale_matrix)
    return scene


def _selfcheck() -> int:
    cube = trimesh.creation.box(extents=(2.0, 3.0, 1.0))
    scene = trimesh.Scene([cube])
    normalize_scene(scene)
    mins, maxs = scene.bounds
    ok = (
        abs(mins[0] - (-0.5)) < 1e-6
        and abs(maxs[0] - 0.5) < 1e-6
        and abs(mins[1] - (-0.5)) < 1e-6
        and abs(maxs[1] - 0.5) < 1e-6
    )
    print(f"selfcheck bounds: {scene.bounds.tolist()} -> {'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


def main() -> int:
    if len(sys.argv) == 2 and sys.argv[1] == "--selfcheck":
        return _selfcheck()
    if not 2 <= len(sys.argv) <= 3:
        print(__doc__)
        return 1
    src = pathlib.Path(sys.argv[1])
    dst = pathlib.Path(sys.argv[2]) if len(sys.argv) == 3 else src.with_suffix(".glb")
    scene = trimesh.load(src, force="scene")
    normalize_scene(scene)
    scene.export(dst)
    print(f"wrote {dst} ({dst.stat().st_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
