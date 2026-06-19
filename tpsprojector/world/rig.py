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


def make_ring_rig(n: int = 6, hfov_deg: float = 75.0, radius: float = 0.3,
                  mount_height: float = 0.5, width: int = 320,
                  height: int = 240) -> List[PinholeCamera]:
    """Build ``n`` outward-facing cameras in a ring about the rig origin."""
    cams: List[PinholeCamera] = []
    for i in range(n):
        theta = 2.0 * np.pi * i / n
        ct, st = np.cos(theta), np.sin(theta)
        # camera axes expressed in the rig frame:
        #   +Z forward = radial outward, +Y = world-down, +X = right (Y x Z)
        fwd = np.array([ct, st, 0.0])
        down = np.array([0.0, 0.0, -1.0])
        right = np.cross(down, fwd)
        R = np.column_stack([right, down, fwd])
        t = np.array([radius * ct, radius * st, mount_height])
        cams.append(PinholeCamera.from_fov(width, height, hfov_deg, Pose(R=R, t=t)))
    return cams
