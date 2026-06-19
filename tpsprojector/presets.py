"""Cinematic virtual-camera presets and smooth transitions.

A :class:`Shot` is an ``(eye, target)`` pair in the rig frame; its
:meth:`Shot.pose` builds the virtual camera pose via ``look_at``. Switching
presets is a smoothstep-eased :func:`tween` of eye and target (interpolating the
look points keeps every intermediate pose valid, no rotation interpolation
needed).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np

from .blend import smoothstep
from .transforms import Pose, look_at


@dataclass
class Shot:
    eye: np.ndarray
    target: np.ndarray

    def __post_init__(self) -> None:
        self.eye = np.asarray(self.eye, dtype=float).reshape(3)
        self.target = np.asarray(self.target, dtype=float).reshape(3)

    def pose(self) -> Pose:
        return look_at(self.eye, self.target)


_PRESETS: Dict[str, Shot] = {
    "behind": Shot(eye=[-4.0, 0.0, 2.5], target=[2.0, 0.0, 0.3]),
    "top_down": Shot(eye=[0.0, 0.0, 8.0], target=[0.0, 0.001, 0.0]),
    "three_quarter_left": Shot(eye=[-3.5, 3.5, 3.0], target=[1.5, 0.0, 0.3]),
    "three_quarter_right": Shot(eye=[-3.5, -3.5, 3.0], target=[1.5, 0.0, 0.3]),
}

PRESET_NAMES = list(_PRESETS.keys())


def get_preset(name: str) -> Shot:
    p = _PRESETS[name]
    return Shot(eye=p.eye.copy(), target=p.target.copy())


def tween(a: Shot, b: Shot, s: float) -> Shot:
    """Smoothstep-eased interpolation between two shots; ``s`` clamped to [0,1]."""
    w = float(smoothstep(np.array(s)))
    return Shot(eye=a.eye + (b.eye - a.eye) * w,
                target=a.target + (b.target - a.target) * w)
