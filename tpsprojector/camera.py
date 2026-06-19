"""Pinhole camera model.

Convention (standard computer vision): in the camera's local frame the optical
axis is ``+Z``, ``+X`` points right, ``+Y`` points down. The camera's
:class:`~tpsprojector.transforms.Pose` maps camera-local coordinates into the
reference (world / rig) frame, so its translation is the camera center.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import Pose


@dataclass
class PinholeCamera:
    K: np.ndarray
    pose: Pose
    width: int
    height: int

    def __post_init__(self) -> None:
        self.K = np.asarray(self.K, dtype=float).reshape(3, 3)

    @classmethod
    def from_fov(
        cls, width: int, height: int, hfov_deg: float, pose: Pose
    ) -> "PinholeCamera":
        """Build a camera from horizontal field of view; square pixels."""
        f = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
        K = np.array(
            [[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]]
        )
        return cls(K=K, pose=pose, width=int(width), height=int(height))

    def project(self, pts_world: np.ndarray):
        """Project world points to pixels.

        Returns ``(uv, valid)`` where ``uv`` is ``(N, 2)`` and ``valid`` marks
        points in front of the camera (``z_cam > 0``).
        """
        pts_world = np.asarray(pts_world, dtype=float).reshape(-1, 3)
        cam = self.pose.inverse().transform_points(pts_world)
        z = cam[:, 2]
        valid = z > 1e-9
        zc = np.where(valid, z, 1.0)
        x = self.K[0, 0] * cam[:, 0] / zc + self.K[0, 2]
        y = self.K[1, 1] * cam[:, 1] / zc + self.K[1, 2]
        uv = np.stack([x, y], axis=-1)
        return uv, valid

    def unproject(self, uv: np.ndarray):
        """Return ``(origins, dirs)`` world-frame rays for pixels ``(N, 2)``.

        Directions are unit vectors; origins are all the camera center.
        """
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        x = (uv[:, 0] - self.K[0, 2]) / self.K[0, 0]
        y = (uv[:, 1] - self.K[1, 2]) / self.K[1, 1]
        dirs_cam = np.stack([x, y, np.ones_like(x)], axis=-1)
        dirs_world = self.pose.transform_dirs(dirs_cam)
        dirs_world /= np.linalg.norm(dirs_world, axis=-1, keepdims=True)
        origins = np.broadcast_to(self.pose.t, dirs_world.shape).copy()
        return origins, dirs_world

    def vfov_deg(self) -> float:
        """Vertical field of view in degrees, from the intrinsics."""
        return float(np.degrees(2.0 * np.arctan((self.height / 2.0) / self.K[1, 1])))

    def backproject(self, uv: np.ndarray, depth: np.ndarray) -> np.ndarray:
        """Back-project pixels ``(N, 2)`` at camera-space ``depth`` (z) to world.

        Inverse of :meth:`project`: turns a pixel plus its forward distance into
        a 3D world point.
        """
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        z = np.asarray(depth, dtype=float).reshape(-1)
        x = (uv[:, 0] - self.K[0, 2]) / self.K[0, 0] * z
        y = (uv[:, 1] - self.K[1, 2]) / self.K[1, 1] * z
        cam = np.stack([x, y, z], axis=-1)
        return self.pose.transform_points(cam)

    def in_bounds(self, uv: np.ndarray) -> np.ndarray:
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        return (
            (uv[:, 0] >= 0)
            & (uv[:, 0] <= self.width - 1)
            & (uv[:, 1] >= 0)
            & (uv[:, 1] <= self.height - 1)
        )

    def in_front(self, pts_world: np.ndarray) -> np.ndarray:
        pts_world = np.asarray(pts_world, dtype=float).reshape(-1, 3)
        cam = self.pose.inverse().transform_points(pts_world)
        return cam[:, 2] > 1e-9
