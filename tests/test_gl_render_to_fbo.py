import numpy as np
import pytest

from tpsprojector.gl_context import gl_available
from tpsprojector.camera import PinholeCamera
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene
from tpsprojector.depth_renderer import synthetic_frames

pytestmark = pytest.mark.skipif(not gl_available(), reason="no GL/EGL context available")


def _setup(width=96, height=72):
    scene = default_scene()
    cameras = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25,
                            mount_height=0.55, tilt_deg=10.0, width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cameras)]
    vc = PinholeCamera.from_fov(width, height, 70.0,
                                look_at(eye=[0.0, -3.0, 2.0], target=[0.0, 0.0, 0.0]))
    return images, cameras, vc


def test_render_to_fbo_readback_matches_render():
    from tpsprojector.gl_renderer import GLBowlRenderer
    images, cameras, vc = _setup()
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    gl = GLBowlRenderer()
    frame, valid = gl.render(images, cameras, surf, vc)

    fbo = gl._render_to_fbo(images, cameras, surf, vc)
    raw = np.frombuffer(fbo.read(components=4, dtype="f4"), dtype="f4").reshape(vc.height, vc.width, 4)
    raw = np.flipud(raw).copy()
    fbo_frame = raw[..., :3].astype(np.float64)
    fbo_valid = raw[..., 3] > 0.5
    fbo_frame[~fbo_valid] = gl.fill_color

    assert np.array_equal(fbo_valid, valid)
    assert np.allclose(fbo_frame, frame, atol=1e-6)
