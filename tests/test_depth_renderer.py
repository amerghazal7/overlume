import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.depth_renderer import CameraFrame, DepthRenderer, synthetic_frames
from tpsprojector.renderer import NumpyRenderer
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import Pose, look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene


def _psnr(a, b, mask=None):
    if mask is not None:
        a, b = a[mask], b[mask]
    mse = np.mean((a - b) ** 2)
    return 99.0 if mse <= 1e-12 else 10.0 * np.log10(1.0 / mse)


def test_render_reproduces_a_single_frontal_frame():
    cam = PinholeCamera.from_fov(32, 32, 90.0, Pose.identity())
    rng = np.random.RandomState(0)
    img = rng.rand(32, 32, 3)
    depth = np.full((32, 32), 5.0)  # frontal plane 5m ahead
    fr = CameraFrame(image=img, depth=depth, camera=cam)

    out, valid = DepthRenderer(splat_radius=0).render([fr], cam)
    assert valid.all()
    np.testing.assert_allclose(out, img, atol=1e-6)


def test_nearer_point_wins_zbuffer():
    cam = PinholeCamera.from_fov(16, 16, 90.0, Pose.identity())
    near = CameraFrame(image=np.tile([1.0, 0.0, 0.0], (16, 16, 1)),
                       depth=np.full((16, 16), 3.0), camera=cam)
    far = CameraFrame(image=np.tile([0.0, 0.0, 1.0], (16, 16, 1)),
                      depth=np.full((16, 16), 8.0), camera=cam)
    out, valid = DepthRenderer(splat_radius=0).render([near, far], cam)
    assert valid.all()
    np.testing.assert_allclose(out[8, 8], [1.0, 0.0, 0.0])  # nearer (red) wins


def test_synthetic_frames_carry_depth():
    scene = default_scene()
    cams = make_ring_rig(n=6, width=80, height=60)
    frames = synthetic_frames(scene, cams)
    assert len(frames) == 6
    assert frames[0].depth.shape == (60, 80)
    assert np.isfinite(frames[0].depth).any()


def test_depth_render_beats_bowl_on_oblique_view():
    # The view that ghosts worst under the bowl. With true depth it should be
    # dramatically closer to ground truth.
    scene = default_scene()
    cams = make_ring_rig(n=6, width=200, height=150)
    frames = synthetic_frames(scene, cams)
    images = [f.image for f in frames]
    virtual = PinholeCamera.from_fov(
        200, 150, 70.0, look_at(eye=[-6.0, 0.0, 4.5], target=[4.0, 0.0, 0.5]))
    truth, _ = scene.render(virtual)

    depth_frame, dvalid = DepthRenderer(splat_radius=1).render(frames, virtual)
    bowl_frame, bvalid = NumpyRenderer().render(
        images, cams, BowlSurface(8, 0.05, 25), virtual)

    # depth fixes ghosting; residual error is honest disocclusion holes (data no
    # camera saw), so it dominates the bowl by a wide margin on the valid region.
    psnr_depth = _psnr(depth_frame, truth, dvalid)
    psnr_bowl = _psnr(bowl_frame, truth, bvalid)
    assert psnr_depth > psnr_bowl + 4.0
    assert psnr_depth > 20.0
