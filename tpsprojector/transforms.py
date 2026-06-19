"""SE(3) rigid transforms and rotation helpers.

A :class:`Pose` represents a rigid transform that maps points from a *local*
frame into a *reference* frame: ``p_ref = R @ p_local + t``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def look_at(eye, target, world_up=(0.0, 0.0, 1.0)) -> "Pose":
    """Build a camera pose looking from ``eye`` toward ``target``.

    Uses the CV camera convention: ``+Z`` points at the target, ``+X`` right,
    ``+Y`` down. Falls back to an alternate up vector when looking nearly
    straight along ``world_up`` (e.g. a top-down view).
    """
    eye = np.asarray(eye, dtype=float).reshape(3)
    target = np.asarray(target, dtype=float).reshape(3)
    f = target - eye
    f /= np.linalg.norm(f)
    up = np.asarray(world_up, dtype=float).reshape(3)
    right = np.cross(f, up)
    if np.linalg.norm(right) < 1e-6:  # looking along world_up: pick another up
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(f, up)
    right /= np.linalg.norm(right)
    down = np.cross(f, right)
    R = np.column_stack([right, down, f])
    return Pose(R=R, t=eye)


@dataclass
class Pose:
    """Rigid transform local->reference: ``p_ref = R @ p_local + t``."""

    R: np.ndarray
    t: np.ndarray

    def __post_init__(self) -> None:
        self.R = np.asarray(self.R, dtype=float).reshape(3, 3)
        self.t = np.asarray(self.t, dtype=float).reshape(3)

    @classmethod
    def identity(cls) -> "Pose":
        return cls(R=np.eye(3), t=np.zeros(3))

    def transform_points(self, pts: np.ndarray) -> np.ndarray:
        """Transform an ``(N, 3)`` array of points local->reference."""
        pts = np.asarray(pts, dtype=float)
        return pts @ self.R.T + self.t

    def transform_dirs(self, dirs: np.ndarray) -> np.ndarray:
        """Rotate ``(N, 3)`` directions (translation ignored)."""
        dirs = np.asarray(dirs, dtype=float)
        return dirs @ self.R.T

    def inverse(self) -> "Pose":
        Rt = self.R.T
        return Pose(R=Rt, t=-Rt @ self.t)

    def compose(self, other: "Pose") -> "Pose":
        """Return the pose equivalent to applying ``self`` after ``other``."""
        return Pose(R=self.R @ other.R, t=self.R @ other.t + self.t)

    def matrix(self) -> np.ndarray:
        M = np.eye(4)
        M[:3, :3] = self.R
        M[:3, 3] = self.t
        return M
