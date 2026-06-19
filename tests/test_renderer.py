import numpy as np

from tpsprojector.camera import PinholeCamera
from tpsprojector.renderer import NumpyRenderer, bilinear_sample
from tpsprojector.surface import BowlSurface
from tpsprojector.transforms import look_at
from tpsprojector.world.rig import make_ring_rig
from tpsprojector.world.scene import default_scene


def _psnr(a, b, mask=None):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    if mask is not None:
        a, b = a[mask], b[mask]
    mse = np.mean((a - b) ** 2)
    if mse <= 1e-12:
        return 99.0
    return 10.0 * np.log10(1.0 / mse)


def test_bilinear_sample_exact_at_integer_coords():
    img = np.arange(3 * 3 * 3, dtype=float).reshape(3, 3, 3) / 30.0
    uv = np.array([[1.0, 2.0], [0.0, 0.0]])  # (x=col, y=row)
    out = bilinear_sample(img, uv)
    np.testing.assert_allclose(out[0], img[2, 1])
    np.testing.assert_allclose(out[1], img[0, 0])


def test_bilinear_sample_midpoint_average():
    img = np.zeros((2, 2, 3))
    img[0, 0] = [0.0, 0.0, 0.0]
    img[0, 1] = [1.0, 1.0, 1.0]
    out = bilinear_sample(img, np.array([[0.5, 0.0]]))
    np.testing.assert_allclose(out[0], [0.5, 0.5, 0.5])


def test_render_output_shape_and_validity():
    cams = make_ring_rig(n=6, width=160, height=120)
    scene = default_scene()
    images = [scene.render(c)[0] for c in cams]
    surf = BowlSurface(R0=8.0, k=0.05, Rmax=25.0)
    virtual = PinholeCamera.from_fov(
        160, 120, 70.0, look_at(eye=[-5.0, 0.0, 4.0], target=[3.0, 0.0, 0.5]))
    r = NumpyRenderer()
    frame, valid = r.render(images, cams, surf, virtual)
    assert frame.shape == (120, 160, 3)
    assert valid.shape == (120, 160)
    assert valid.mean() > 0.6  # most pixels resolve to a camera


def test_reprojection_reproduces_a_source_camera():
    # Parallax-free correctness check: synthesizing at a real camera's own pose
    # must reproduce that camera's own image where the bowl is hit. This isolates
    # the reproject<->unproject<->surface math from the bowl's fidelity limits.
    cams = make_ring_rig(n=6, width=200, height=150)
    scene = default_scene()
    images = [scene.render(c)[0] for c in cams]
    surf = BowlSurface(R0=8.0, k=0.05, Rmax=25.0)

    frame, valid = NumpyRenderer().render(images, cams, surf, cams[0])
    assert valid.mean() > 0.3
    assert _psnr(frame, images[0], valid) > 22.0


def test_synthesis_tracks_ground_truth_for_overhead_view():
    # Overhead near-rig view (small parallax, ground-dominated) should track the
    # true view comfortably better than a constant-gray baseline.
    cams = make_ring_rig(n=6, width=160, height=160)
    scene = default_scene()
    images = [scene.render(c)[0] for c in cams]
    surf = BowlSurface(R0=8.0, k=0.05, Rmax=25.0)
    virtual = PinholeCamera.from_fov(
        160, 160, 80.0, look_at(eye=[0.0, 0.0, 6.0], target=[0.0, 0.001, 0.0]))

    frame, valid = NumpyRenderer().render(images, cams, surf, virtual)
    truth, _ = scene.render(virtual)
    assert _psnr(frame, truth, valid) > 18.0
    assert _psnr(frame, truth, valid) > _psnr(np.full_like(truth, 0.5), truth, valid)
