import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.transforms import look_at
from tpsprojector.depth_renderer import DepthRenderer, synthetic_frames
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


def test_gl_depth_matches_numpy_depth():
    from tpsprojector.gl_depth_renderer import GLDepthRenderer
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    frames = synthetic_frames(scene, cameras)
    vc = PinholeCamera.from_fov(96, 72, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))

    gl_frame, gl_valid = GLDepthRenderer(splat_radius=1).render(frames, vc)
    np_frame, np_valid = DepthRenderer(splat_radius=1).render(frames, vc)
    assert gl_frame.shape == (72, 96, 3)
    # splat footprints are square in both; masks agree on most pixels
    assert (gl_valid == np_valid).mean() > 0.90
    both = gl_valid & np_valid
    assert both.sum() > 1000
    assert _psnr(gl_frame[both], np_frame[both]) > 28.0
