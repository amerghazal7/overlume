"""Export golden fixture for the C++ GTest golden test.

Renders the bowl reference via NumpyRenderer using the same setup as
test_cuda_parity.py::test_cuda_bowl_matches_numpy, then writes the fixture
files to cuda/tests/golden/.

Run from the repository root:
    python3 cuda/tools/export_golden.py

Outputs:
    cuda/tests/golden/manifest.txt  -- scalar metadata
    cuda/tests/golden/cameras.txt   -- N lines, K(9)+R(9)+t(3)+w+h
    cuda/tests/golden/vcam.txt      -- 1 line, same 23-field format
    cuda/tests/golden/images.bin    -- float32, N*camH*camW*3
    cuda/tests/golden/golden_bowl.bin -- float32, OH*OW*3
"""

from __future__ import annotations

import os
import sys
import struct
import numpy as np

# ---- make sure tpsprojector is importable from repo root -------------------
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames
from tpsprojector.renderer import NumpyRenderer

# ---------------------------------------------------------------------------
# Setup: IDENTICAL to test_cuda_parity.py::_bowl_setup (W=96, H=72)
# ---------------------------------------------------------------------------
OUT_W, OUT_H = 96, 72
CAM_W, CAM_H = 128, 96

scene = default_scene()
cams = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25, mount_height=0.55,
                     tilt_deg=10.0, width=CAM_W, height=CAM_H)
images = [f.image for f in synthetic_frames(scene, cams)]
vc = PinholeCamera.from_fov(OUT_W, OUT_H, 70.0,
                             look_at(eye=[0, -3, 2], target=[0, 0, 0]))
surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)

N = len(cams)

# ---------------------------------------------------------------------------
# Render reference
# ---------------------------------------------------------------------------
print(f"Rendering NumpyRenderer reference ({OUT_W}x{OUT_H}) ...", flush=True)
np_frame, np_valid = NumpyRenderer().render(images, cams, surf, vc)
print(f"  valid pixels: {np_valid.sum()} / {OUT_W * OUT_H}")

# ---------------------------------------------------------------------------
# Write fixture
# ---------------------------------------------------------------------------
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "tests", "golden")
os.makedirs(OUT_DIR, exist_ok=True)
print(f"Writing to {OUT_DIR} ...", flush=True)

# -- manifest.txt -----------------------------------------------------------
with open(os.path.join(OUT_DIR, "manifest.txt"), "w") as f:
    f.write(f"ncam {N}\n")
    f.write(f"cam_w {CAM_W}\n")
    f.write(f"cam_h {CAM_H}\n")
    f.write(f"out_w {OUT_W}\n")
    f.write(f"out_h {OUT_H}\n")
    f.write(f"R0 {surf.R0}\n")
    f.write(f"k {surf.k}\n")
    f.write(f"Rmax {surf.Rmax}\n")


def cam_line(cam: PinholeCamera) -> str:
    """23 fields: K(9) + R(9) + t(3) + w + h (space-separated)."""
    K = np.asarray(cam.K, dtype=float).ravel()          # (9,)
    R = np.asarray(cam.pose.R, dtype=float).ravel()     # (9,)
    t = np.asarray(cam.pose.t, dtype=float).ravel()     # (3,)
    parts = (
        list(K) + list(R) + list(t) + [cam.width, cam.height]
    )
    return " ".join(str(v) for v in parts)


# -- cameras.txt ------------------------------------------------------------
with open(os.path.join(OUT_DIR, "cameras.txt"), "w") as f:
    for cam in cams:
        f.write(cam_line(cam) + "\n")

# -- vcam.txt ---------------------------------------------------------------
with open(os.path.join(OUT_DIR, "vcam.txt"), "w") as f:
    f.write(cam_line(vc) + "\n")

# -- images.bin -------------------------------------------------------------
# Shape: (N, camH, camW, 3), float32, row-major
imgs_arr = np.stack([np.asarray(img, dtype=np.float32) for img in images])  # (N,H,W,3)
assert imgs_arr.shape == (N, CAM_H, CAM_W, 3), f"Unexpected shape {imgs_arr.shape}"
imgs_arr.tofile(os.path.join(OUT_DIR, "images.bin"))
print(f"  images.bin: {imgs_arr.nbytes / 1024:.1f} KB")

# -- golden_bowl.bin --------------------------------------------------------
# Shape: (OH, OW, 3), float32
frame_f32 = np.asarray(np_frame, dtype=np.float32)  # (OH, OW, 3)
assert frame_f32.shape == (OUT_H, OUT_W, 3), f"Unexpected shape {frame_f32.shape}"
frame_f32.tofile(os.path.join(OUT_DIR, "golden_bowl.bin"))
print(f"  golden_bowl.bin: {frame_f32.nbytes / 1024:.1f} KB")

# -- summary ----------------------------------------------------------------
total_kb = (imgs_arr.nbytes + frame_f32.nbytes) / 1024
print(f"  Total binary fixture size: {total_kb:.1f} KB")
print("Done.")
