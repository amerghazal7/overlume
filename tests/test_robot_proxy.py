import os
import sys

import numpy as np
import pytest

# tpscuda is installed under cuda/install/libs/rendering_reprojector/python
_CUDA_PY = os.path.join(os.path.dirname(__file__), "..", "cuda", "install",
                        "libs", "rendering_reprojector", "python")
if os.path.isdir(_CUDA_PY):
    sys.path.insert(0, _CUDA_PY)


def cuda_available():
    try:
        import tpscuda  # noqa
        return True
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not cuda_available(), reason="tpscuda module not built")

# OBJ->rig used for M02P (obj: x-fwd, y-up, z-right; rig: x-fwd, y-left, z-up)
T_M02P = np.array([1, 0, 0,  0, 0, -1,  0, 1, 0,  0, 0, 0], dtype="f4")
# Loader's fixed light direction (rig frame), keep in sync with mesh_loader.cpp
_L = np.array([0.3, 0.15, 0.94]) / np.linalg.norm([0.3, 0.15, 0.94])


def _write_quad_obj(tmp_path, with_mtl=True):
    if with_mtl:
        (tmp_path / "m.mtl").write_text("newmtl red\nKd 1.0 0.0 0.0\n")
    (tmp_path / "m.obj").write_text(
        "mtllib m.mtl\n"
        "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
        "vn 0 1 0\n"
        "usemtl red\n"
        "f 1//1 2//1 3//1 4//1\n")
    return str(tmp_path / "m.obj")


def test_load_obj_quad_triangulated_and_transformed(tmp_path):
    import tpscuda
    verts, cols = tpscuda.load_obj_mesh(_write_quad_obj(tmp_path), T_M02P)
    assert verts.shape == (2, 9) and cols.shape == (2, 3)
    # fan triangulation from corner 0: (v1,v2,v3), (v1,v3,v4)
    # obj (1,1,0) -> rig (1, 0, 1)
    assert np.allclose(verts[0], [0, 0, 0, 1, 0, 0, 1, 0, 1], atol=1e-6)
    assert np.allclose(verts[1], [0, 0, 0, 1, 0, 1, 0, 0, 1], atol=1e-6)
    # normal obj (0,1,0) -> rig (0,0,1); lambert = 0.35 + 0.65*|dot(n,L)|
    shade = 0.35 + 0.65 * abs(_L[2])
    assert np.allclose(cols[0], [shade, 0, 0], atol=1e-5)
    assert np.allclose(cols[1], cols[0])


def test_load_obj_missing_mtl_falls_back_gray(tmp_path):
    import tpscuda
    verts, cols = tpscuda.load_obj_mesh(_write_quad_obj(tmp_path, with_mtl=False),
                                        T_M02P)
    shade = 0.35 + 0.65 * abs(_L[2])
    assert np.allclose(cols[0], np.array([0.7, 0.7, 0.7]) * shade, atol=1e-5)


def test_load_obj_missing_file_raises(tmp_path):
    import tpscuda
    with pytest.raises(RuntimeError):
        tpscuda.load_obj_mesh(str(tmp_path / "nope.obj"), T_M02P)


def _down_vcam(W=64, H=48, fx=40.0, z=5.0):
    """vcam looking straight down from (0,0,z): R columns = right/down/fwd."""
    # right=(0,-1,0), down=(-1,0,0), fwd=(0,0,-1); fwd == right x down (CV frame)
    R = np.array([[0, -1, 0],
                  [-1, 0, 0],
                  [0, 0, -1]], dtype="f4")
    K = np.array([[fx, 0, W / 2], [0, fx, H / 2], [0, 0, 1]], dtype="f4")
    return dict(K=K.ravel(), R=R.ravel(), t=np.array([0, 0, z], "f4"),
                width=W, height=H)


def test_robot_triangle_composited_over_env():
    import tpscuda
    W, H = 64, 48
    r = tpscuda.Reprojector(W, H)
    vcam = _down_vcam(W, H)
    # single red triangle on the z=1 plane near the origin
    verts = np.array([[0, 0, 1, 0.5, 0, 1, 0, 0.5, 1]], dtype="f4")
    cols = np.array([[1.0, 0.0, 0.0]], dtype="f4")
    r.upload_robot_mesh(verts, cols)
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0)
    # no cameras -> env alpha 0 everywhere except the robot
    # projected corners: (32,24), (32,19), (27,24) -> interior pixels around (30,22)
    patch = out[20:24, 28:32]
    assert np.any(patch[..., 3] == 1.0), "robot not rendered"
    hit = patch[patch[..., 3] == 1.0]
    assert np.allclose(hit[:, 0], 1.0) and np.allclose(hit[:, 1:3], 0.0), \
        "robot pixels are not the uploaded color"
    assert out[5, 5, 3] == 0.0, "robot leaked outside its projection"
    # clearing the mesh restores env-only output
    r.upload_robot_mesh(np.zeros((0, 9), "f4"), np.zeros((0, 3), "f4"))
    out2 = r.render_bowl(vcam, 6.0, 0.08, 20.0)
    assert np.all(out2[..., 3] == 0.0)


def test_robot_depth_test_nearer_triangle_wins():
    import tpscuda
    W, H = 64, 48
    r = tpscuda.Reprojector(W, H)
    vcam = _down_vcam(W, H)
    # same right-angle footprint; z=1 is 4 m from the cam, z=2 is 3 m (nearer)
    tri = np.array([0, 0, 0, 0.5, 0, 0, 0, 0.5, 0], "f4").reshape(3, 3)
    far = (tri + [0, 0, 1.0]).ravel()
    near = (tri + [0, 0, 2.0]).ravel()
    verts = np.stack([far, near]).astype("f4")
    cols = np.array([[1, 0, 0], [0, 1, 0]], dtype="f4")
    r.upload_robot_mesh(verts, cols)
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0)
    # both triangles share the right-angle vertex at the principal point (32,24)
    # and open toward -x/-y on screen (legs: far 5 px, near ~6.7 px); near fully
    # encloses far, so any pixel in far's footprint, e.g. (y=23,x=31), is covered
    # by both -- the nearer (green) triangle must win the depth test there
    px = out[23, 31]
    assert px[3] == 1.0
    assert px[1] > 0.9 and px[0] < 0.1, f"nearer triangle lost the depth test: {px}"
