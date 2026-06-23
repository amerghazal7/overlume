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
    import tpscuda
    r = tpscuda.Reprojector(8, 6)
    vcam = dict(K=np.eye(3, dtype="f4").ravel(), R=np.eye(3, dtype="f4").ravel(),
                t=np.zeros(3, "f4"), width=8, height=6)
    out = r.render_bowl(vcam, 6.0, 0.08, 20.0)
    assert out.shape == (6, 8, 4)
    assert np.allclose(out[..., :3], [0.1, 0.2, 0.3], atol=1e-4)  # fill kernel constant
    assert np.allclose(out[..., 3], 1.0)
