import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import FlatSurface
from tpsprojector.transforms import look_at
from tpsprojector.renderer import NumpyRenderer
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


def _setup(width=96, height=72):
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0,
                            width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cameras)]
    vc = PinholeCamera.from_fov(width, height, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    return images, cameras, vc


def test_gl_bowl_matches_numpy_on_flat_surface():
    from tpsprojector.gl_renderer import GLBowlRenderer
    images, cameras, vc = _setup()
    surf = FlatSurface(z0=0.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    np_frame, np_valid = NumpyRenderer().render(images, cameras, surf, vc)
    assert gl_frame.shape == np_frame.shape == (vc.height, vc.width, 3)
    # masks agree on the vast majority of pixels (edge rounding aside)
    assert (gl_valid == np_valid).mean() > 0.97
    both = gl_valid & np_valid
    assert both.sum() > 1000
    assert _psnr(gl_frame[both], np_frame[both]) > 40.0


def test_gl_bowl_matches_numpy_on_bowl_surface():
    from tpsprojector.gl_renderer import GLBowlRenderer
    from tpsprojector.surface import BowlSurface
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    np_frame, np_valid = NumpyRenderer().render(images, cameras, surf, vc)
    assert (gl_valid == np_valid).mean() > 0.97
    both = gl_valid & np_valid
    assert both.sum() > 1000
    assert _psnr(gl_frame[both], np_frame[both]) > 40.0


def test_gl_bowl_meets_ground_truth_threshold():
    from tpsprojector.gl_renderer import GLBowlRenderer
    from tpsprojector.surface import BowlSurface
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl_frame, gl_valid = GLBowlRenderer().render(images, cameras, surf, vc)
    truth, _ = default_scene().render(vc)
    # same regime as the NumPy integration test: above the floor on overlap
    assert _psnr(gl_frame[gl_valid], truth[gl_valid]) > 12.0
