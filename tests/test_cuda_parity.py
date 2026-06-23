import os
import numpy as np
import pytest

# tpscuda is installed under cuda/install/libs/rendering_reprojector/python
_CUDA_PY = os.path.join(os.path.dirname(__file__), "..", "cuda", "install",
                        "libs", "rendering_reprojector", "python")
import sys
if os.path.isdir(_CUDA_PY):
    sys.path.insert(0, _CUDA_PY)


def cuda_available():
    try:
        import tpscuda  # noqa
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not cuda_available(), reason="tpscuda module not built")


def test_toolchain_roundtrip():
    """Smoke test: render_bowl returns correct shape and finite values.

    The original fill-constant assertions (0.1, 0.2, 0.3, 1.0) were removed in
    Task 3 because render_bowl now runs the real reprojection kernel.  This test
    verifies shape, dtype, and finiteness only; the full correctness gate is
    test_cuda_bowl_matches_numpy below.
    """
    import tpscuda
    r = tpscuda.Reprojector(8, 6)
    vcam = dict(K=np.eye(3, dtype="f4").ravel(), R=np.eye(3, dtype="f4").ravel(),
                t=np.zeros(3, "f4"), width=8, height=6)
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0)
    assert out.shape == (6, 8, 4)
    assert np.all(np.isfinite(out)), "render_bowl output contains non-finite values"
    # Alpha must be 0 or 1 for every pixel.
    assert np.all((out[..., 3] == 0.0) | (out[..., 3] == 1.0)), \
        "alpha channel must be 0 or 1"


# ---------------------------------------------------------------------------
# Bowl parity: CUDA render_bowl must match NumpyRenderer to PSNR > 40 dB
# (mask agreement > 0.97 on jointly-valid pixels)
# ---------------------------------------------------------------------------

def _bowl_setup(W=96, H=72):
    from tpsprojector.camera import PinholeCamera
    from tpsprojector.surface import BowlSurface
    from tpsprojector.transforms import look_at
    from tpsprojector.world.rig import make_ring_rig
    from tpsprojector.world.scene import default_scene
    from tpsprojector.depth_renderer import synthetic_frames
    scene = default_scene()
    cams = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25, mount_height=0.55,
                         tilt_deg=10.0, width=128, height=96)
    images = [f.image for f in synthetic_frames(scene, cams)]
    vc = PinholeCamera.from_fov(W, H, 70.0, look_at(eye=[0, -3, 2], target=[0, 0, 0]))
    surf = BowlSurface(R0=6.0, k=0.08, Rmax=20.0)
    return images, cams, vc, surf


def _cam_dict(cam):
    return dict(K=np.asarray(cam.K, "f4").ravel(), R=np.asarray(cam.pose.R, "f4").ravel(),
                t=np.asarray(cam.pose.t, "f4"), width=cam.width, height=cam.height)


def _psnr(a, b):
    mse = float(np.mean((np.asarray(a) - np.asarray(b)) ** 2))
    return 99.0 if mse < 1e-12 else 10.0 * np.log10(1.0 / mse)


def test_cuda_bowl_matches_numpy():
    import tpscuda
    from tpsprojector.renderer import NumpyRenderer
    images, cams, vc, surf = _bowl_setup()
    r = tpscuda.Reprojector(vc.width, vc.height)
    r.set_cameras([_cam_dict(c) for c in cams])
    r.upload_images(np.stack([np.asarray(im, "f4") for im in images]))  # (N,H,W,3)
    out = r.render_bowl(_cam_dict(vc), surf.R0, surf.k, surf.Rmax)
    cuda_frame, cuda_valid = out[..., :3].astype(float), out[..., 3] > 0.5
    np_frame, np_valid = NumpyRenderer().render(images, cams, surf, vc)
    assert (cuda_valid == np_valid).mean() > 0.97
    both = cuda_valid & np_valid
    assert _psnr(cuda_frame[both], np_frame[both]) > 40.0


def test_cuda_depth_matches_numpy():
    import tpscuda
    from tpsprojector.depth_renderer import DepthRenderer, synthetic_frames
    from tpsprojector.world.rig import make_ring_rig
    from tpsprojector.world.scene import default_scene
    from tpsprojector.camera import PinholeCamera
    from tpsprojector.transforms import look_at
    scene = default_scene()
    cams = make_ring_rig(n=6, hfov_deg=85.0, radius=0.25, mount_height=0.55,
                         tilt_deg=10.0, width=128, height=96)
    frames = synthetic_frames(scene, cams)
    vc = PinholeCamera.from_fov(96, 72, 70.0, look_at(eye=[0, -3, 2], target=[0, 0, 0]))
    r = tpscuda.Reprojector(96, 72)
    r.set_cameras([_cam_dict(c) for c in cams])
    r.upload_images(np.stack([np.asarray(f.image, "f4") for f in frames]))
    depth = np.stack([np.where(np.isfinite(f.depth), f.depth, np.inf).astype("f4") for f in frames])
    r.upload_depth(depth)
    out = r.render_depth(_cam_dict(vc), 1)
    cf, cvld = out[..., :3].astype(float), out[..., 3] > 0.5
    nf, nvld = DepthRenderer(splat_radius=1).render(frames, vc)
    assert (cvld == nvld).mean() > 0.90
    both = cvld & nvld
    assert _psnr(cf[both], nf[both]) > 28.0
