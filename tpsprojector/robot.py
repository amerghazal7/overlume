"""Robot body proxy and depth compositing.

No real camera sees the robot (they are mounted on it, looking outward), so the
robot's own body in the virtual TPS view must come from a known 3D proxy. We
render the proxy from the virtual pose to RGB + depth and composite it over the
synthesized environment. Because the robot sits at the rig center it is always
nearer than the bowl-projected environment, so it simply occludes wherever it is
drawn (a depth test is kept for correctness and future ghost/translucent modes).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .camera import PinholeCamera
from .world.scene import Scene, _Builder


@dataclass
class RobotProxy:
    scene: Scene  # the proxy geometry, expressed in the rig frame

    @property
    def vertices(self) -> np.ndarray:
        return self.scene.vertices

    @classmethod
    def default(cls, footprint_radius: float = 0.5,
                height: float = 0.5) -> "RobotProxy":
        """A body box of the given footprint radius with a 'front' marker (+X)."""
        r = float(footprint_radius)
        b = _Builder()
        b.add_box(center=(0.0, 0.0, height / 2),
                  size=(2 * r, 1.6 * r, height), color=(0.25, 0.5, 0.8))
        # front marker so orientation is visible in the view
        b.add_box(center=(r + 0.06, 0.0, height * 0.6),
                  size=(0.14, 0.5 * r, 0.2 * height + 0.1), color=(0.95, 0.85, 0.2))
        return cls(scene=b.build())

    def render(self, camera: PinholeCamera):
        """Render the proxy from ``camera`` -> ``(rgb, depth)``.

        ``depth`` is ``inf`` where the robot is absent, so ``isfinite(depth)``
        is the robot mask.
        """
        return self.scene.render(camera, bg_color=(0.0, 0.0, 0.0))


def composite(env_rgb: np.ndarray, robot_rgb: np.ndarray,
              robot_depth: np.ndarray) -> np.ndarray:
    """Composite the robot over the environment where the robot was drawn."""
    out = np.array(env_rgb, dtype=float, copy=True)
    mask = np.isfinite(robot_depth)
    out[mask] = robot_rgb[mask]
    return out
