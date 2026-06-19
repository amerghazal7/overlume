"""The onboard multi-camera rig embodied in the robot.

Cameras are placed in a horizontal ring around the rig origin, evenly spaced and
looking radially outward. With ``n`` cameras the spacing is ``360/n`` degrees; a
horizontal FOV larger than that spacing produces the overlap between neighbours
(default: 6 cameras, 75 deg FOV -> ~15 deg overlap).
"""

from __future__ import annotations

from typing import List

import numpy as np

from ..camera import PinholeCamera
from ..transforms import Pose


def tilt_for_body_edge(mount_height: float, mount_radius: float,
                       body_radius: float, vfov_deg: float) -> float:
    """Downward tilt (deg) so the camera's near edge just reaches the body edge.

    Mirrors how cameras are mounted on a real rig: pitched down just enough that
    the nearest ground they see lands at the robot's body footprint edge (the
    body boundary sits at the bottom of frame), leaving no blind ground ring
    around the robot — but no more tilt than that, to keep the far view. Returns
    0 when the body edge is already within view without tilting.

    Geometry: the bottom FOV ray sits ``vfov/2`` below the optical axis; tilting
    by ``θ`` puts it ``θ + vfov/2`` below horizontal. It hits the ground at
    radius ``mount_radius + mount_height / tan(θ + vfov/2)``. Setting that to
    ``body_radius`` and solving for ``θ`` gives the tilt.
    """
    denom = max(body_radius - mount_radius, 1e-3)
    phi = np.arctan2(mount_height, denom)          # bottom-ray angle below horiz
    tilt = np.degrees(phi) - vfov_deg / 2.0
    return float(max(tilt, 0.0))


def _broadcast(name: str, value, n: int) -> np.ndarray:
    arr = np.atleast_1d(np.asarray(value, dtype=float))
    if arr.size == 1:
        arr = np.repeat(arr, n)
    if arr.size != n:
        raise ValueError(f"{name} must be a scalar or length-{n} sequence, "
                         f"got {arr.size} values")
    return arr


def make_ring_rig(n: int = 6, hfov_deg: float = 75.0, radius: float = 0.3,
                  mount_height=0.5, tilt_deg=0.0, width: int = 320,
                  height: int = 240) -> List[PinholeCamera]:
    """Build ``n`` outward-facing cameras in a ring about the rig origin.

    ``mount_height`` and ``tilt_deg`` may each be a scalar (shared by all
    cameras) or a length-``n`` sequence (per-camera) — real rigs mount cameras
    at different heights, and the body-edge tilt then differs per camera.
    ``tilt_deg`` pitches a camera downward for more near-field ground coverage.
    """
    heights = _broadcast("mount_height", mount_height, n)
    tilts = np.radians(_broadcast("tilt_deg", tilt_deg, n))
    cams: List[PinholeCamera] = []
    for i in range(n):
        theta = 2.0 * np.pi * i / n
        ct, st = np.cos(theta), np.sin(theta)
        td = tilts[i]
        # camera axes in the rig frame: +Z forward = radial outward, pitched
        # down by tilt; +X right = world-up x forward; +Y down = forward x right
        fwd = np.array([ct * np.cos(td), st * np.cos(td), -np.sin(td)])
        fwd /= np.linalg.norm(fwd)
        right = np.cross(np.array([0.0, 0.0, -1.0]), fwd)
        right /= np.linalg.norm(right)
        down = np.cross(fwd, right)
        R = np.column_stack([right, down, fwd])
        t = np.array([radius * ct, radius * st, heights[i]])
        cams.append(PinholeCamera.from_fov(width, height, hfov_deg, Pose(R=R, t=t)))
    return cams
