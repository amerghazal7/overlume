# Robot Proxy Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The live CUDA pipeline composites the real M02P robot mesh (OBJ+MTL) into every rendered virtual-camera frame, with the model path configurable.

**Architecture:** A host-side OBJ+MTL loader in `rendering_reprojector` bakes per-triangle lit colors and rig-frame vertices; a new CUDA triangle-raster kernel draws them into a packed z-buffer reusing splat.cu's `zbuf_init`/`resolve_kernel`; the existing `launch_composite` overlays the robot layer on the environment inside all three `render_*` paths. The ROS node gains `robot_model_path` + `robot_model_transform` params and calls the loader at `on_configure`.

**Tech Stack:** C++17, CUDA, pybind11 (`tpscuda` test bindings), ROS2 Humble (`rclcpp_lifecycle`), pytest.

**Spec:** `docs/superpowers/specs/2026-07-06-robot-proxy-rendering-design.md`.
**Deviation from spec (deliberate):** the OBJ/MTL loader lives in the `rendering_reprojector` lib (not the node) so it is pytest-testable through the existing `tpscuda` bindings; the node stays a thin caller. Same zero-new-deps property.

**Status note:** Task 1 is COMPLETE (commit e1b6bcd) — mesh_loader.hpp/.cpp + CMake + `tpscuda.load_obj_mesh` binding + `tests/test_robot_proxy.py` (3 tests). Execution resumes at Task 2.

## Global Constraints

- No per-frame `cudaMalloc`/`cudaFree`: all device buffers go through `Impl::ensure_bytes` (grow-only). See the warning at the top of `reprojector.cpp` — churning allocations stalls the shared-GPU CARLA server.
- Mesh upload happens once (`on_configure`), never per frame.
- Run Python tests as: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest <file> -v` from the repo root `/home/ag7/Documents/TPSProjector`.
- CUDA lib build: `cd cuda/scripts/libs_build && ./libs_build.sh Release` (installs to `cuda/install/libs`, including the `tpscuda` Python module).
- Node build: `cd cuda/scripts/ros_apps_build && ./colcon_build.sh micropilot_rendering_node`.
- Code style: `namespace micropilot::rendering`, doxygen `/** @file ... */` headers, 4-space indent, brace-on-own-line (match `reprojector.cpp`).
- The model files stay at `/home/ag7/Downloads/M02P.obj` + `/home/ag7/Downloads/M02P.mtl` — NOT committed to git.
- The robot proxy is optional everywhere: empty `robot_model_path` or load failure must leave rendering exactly as it is today.

---

### Task 1: OBJ+MTL mesh loader in the lib (+ bindings + pytest) — COMPLETE (commit e1b6bcd)

Done: `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/mesh_loader.hpp`,
`src/mesh_loader.cpp`, CMake source entry, `tpscuda.load_obj_mesh` binding,
`tests/test_robot_proxy.py` with 3 passing tests. Interfaces produced (used by later tasks):

- C++: `struct RobotMesh { std::vector<float> verts; std::vector<float> cols; std::size_t n_tris; };`
  and `RobotMesh load_obj_mesh(const std::string& obj_path, const float* transform)` in
  `namespace micropilot::rendering` (header `rendering_reprojector/mesh_loader.hpp`).
  `verts` = `n_tris*9` floats (3 rig-frame xyz per triangle), `cols` = `n_tris*3` floats
  (one baked RGB per triangle). `transform` = 12 floats `[R(9 row-major) | t(3)]`, OBJ→rig.
  Throws `std::runtime_error` on unreadable/empty OBJ.
- Python: `tpscuda.load_obj_mesh(path, transform (12,f4)) -> (verts (N,9) f4, cols (N,3) f4)`.

---

### Task 2: Robot raster kernel + upload_robot_mesh + composite in all render paths

**Files:**
- Create: `cuda/src/libs/rendering_reprojector/src/kernels/robot_raster.cu`
- Modify: `cuda/src/libs/rendering_reprojector/src/kernels/camdev.hpp` (declare `launch_robot`)
- Modify: `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/reprojector.hpp` (declare `upload_robot_mesh`)
- Modify: `cuda/src/libs/rendering_reprojector/src/reprojector.cpp` (buffers, upload, composite hook)
- Modify: `cuda/src/libs/rendering_reprojector/CMakeLists.txt` (add the .cu)
- Modify: `cuda/src/libs/rendering_reprojector/bindings/tps_pybind.cpp` (bind `upload_robot_mesh`)
- Test: `tests/test_robot_proxy.py` (append)

**Interfaces:**
- Consumes: `verts (N,9)` / `cols (N,3)` arrays (Task 1's layout), existing `launch_composite`, `zbuf_init`, `resolve_kernel` (splat.cu), `Impl::ensure_bytes`.
- Produces:
  - C++: `void Reprojector::upload_robot_mesh(const float* verts, const float* cols, std::size_t n_tris)` — persistent upload; `n_tris == 0` disables the robot. All three `render_bowl/render_depth/render_hybrid` composite the robot when present (signatures unchanged).
  - CUDA: `void launch_robot(unsigned long long* d_zbuf, float* d_layer, int OW, int OH, const float* d_verts, const float* d_cols, int ntris, CamDev v)` in `camdev.hpp`.
  - Python: `Reprojector.upload_robot_mesh(verts (N,9) f4, cols (N,3) f4)`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_robot_proxy.py`:

```python
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
    # both triangles cover pixel (x=33, y=25) just inside the right angle at the
    # principal point (legs: far 5 px, near ~6.7 px, opening toward +x/+y);
    # the nearer (green) triangle must win the depth test
    px = out[25, 33]
    assert px[3] == 1.0
    assert px[1] > 0.9 and px[0] < 0.1, f"nearer triangle lost the depth test: {px}"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_robot_proxy.py -v -k robot_`
Expected: the two new tests FAIL with `AttributeError: ... no attribute 'upload_robot_mesh'`; Task 1 tests still PASS.

- [ ] **Step 3: Write the raster kernel**

Create `cuda/src/libs/rendering_reprojector/src/kernels/robot_raster.cu`:

```cpp
/** @file robot_raster.cu @brief Robot proxy triangle raster into a packed z-buffer.
 *
 *  Same packing as splat.cu (depth float bits << 32 | index): for z > 0, IEEE
 *  float bits are monotonic, so atomicMin selects the nearest triangle; the low
 *  32 bits index the winning triangle's color. zbuf_init/resolve_kernel are
 *  reused from splat.cu (external linkage; host-side launches only).
 */
#include <cuda_runtime.h>
#include <stdint.h>

#include "kernels/camdev.hpp"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
// defined in splat.cu
__global__ void zbuf_init(unsigned long long* zbuf, int n);
__global__ void resolve_kernel(const unsigned long long* zbuf, const float* cols, float* out,
                               int n, float fr, float fg, float fb);

// One thread per triangle: project the 3 vertices with the virtual camera,
// clamp the screen bbox, test pixel centers with barycentric edge functions
// (signed sub-area / signed area — winding-independent), atomicMin the packed
// affine-interpolated depth. Affine (not perspective-correct) depth across a
// triangle is fine here: the M02P triangles are millimetre-scale.
// ponytail: a near triangle spanning the screen would walk a huge bbox; the
// robot is always >= 1.5 m from the vcam (orbit DIST_MIN), so bboxes stay tiny.
__global__ void robot_raster_kernel(unsigned long long* zbuf, int OW, int OH,
                                    const float* verts, int ntris, CamDev v)
{
    int tri = blockIdx.x * blockDim.x + threadIdx.x;
    if (tri >= ntris) return;
    const float* q = verts + tri * 9;
    float sx[3], sy[3], sz[3];
    for (int k = 0; k < 3; ++k)
    {
        float3 P = make_float3(q[k * 3], q[k * 3 + 1], q[k * 3 + 2]);
        float3 rel = vsub(P, v.t);
        float z = vdot(v.fwd, rel);
        if (z <= 1e-4f) return;  // any vertex at/behind the camera -> drop triangle
        sx[k] = v.fx * vdot(v.right, rel) / z + v.cx;
        sy[k] = v.fy * vdot(v.down, rel) / z + v.cy;
        sz[k] = z;
    }
    float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sy[1] - sy[0]) * (sx[2] - sx[0]);
    if (fabsf(area) < 1e-12f) return;  // degenerate/edge-on
    int x0 = max(0, (int)floorf(fminf(sx[0], fminf(sx[1], sx[2]))));
    int x1 = min(OW - 1, (int)ceilf(fmaxf(sx[0], fmaxf(sx[1], sx[2]))));
    int y0 = max(0, (int)floorf(fminf(sy[0], fminf(sy[1], sy[2]))));
    int y1 = min(OH - 1, (int)ceilf(fmaxf(sy[0], fmaxf(sy[1], sy[2]))));
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
        {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = ((sx[1] - px) * (sy[2] - py) - (sy[1] - py) * (sx[2] - px)) / area;
            float w1 = ((sx[2] - px) * (sy[0] - py) - (sy[2] - py) * (sx[0] - px)) / area;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            float z = w0 * sz[0] + w1 * sz[1] + w2 * sz[2];
            unsigned long long packed =
                ((unsigned long long)__float_as_uint(z) << 32) | (unsigned int)tri;
            atomicMin(&zbuf[y * OW + x], packed);
        }
}

void launch_robot(unsigned long long* d_zbuf, float* d_layer, int OW, int OH,
                  const float* d_verts, const float* d_cols, int ntris, CamDev v)
{
    int n = OW * OH, t = 256;
    zbuf_init<<<(n + t - 1) / t, t>>>(d_zbuf, n);
    robot_raster_kernel<<<(ntris + t - 1) / t, t>>>(d_zbuf, OW, OH, d_verts, ntris, v);
    // fill = black with alpha 0 -> launch_composite keeps the environment there
    resolve_kernel<<<(n + t - 1) / t, t>>>(d_zbuf, d_cols, d_layer, n, 0.0f, 0.0f, 0.0f);
}
}  // namespace micropilot::rendering
```

If linking fails on the cross-TU `zbuf_init`/`resolve_kernel` launches (it should not — the splat.cu definitions have external linkage), the fallback is to copy those two small kernels into this file renamed `robot_zbuf_init`/`robot_resolve_kernel`.

- [ ] **Step 4: Declare in camdev.hpp**

In `cuda/src/libs/rendering_reprojector/src/kernels/camdev.hpp`, after the `launch_composite` declaration:

```cpp
/** Robot proxy triangle raster: z-buffered mesh layer for compositing (robot proxy). */
void launch_robot(unsigned long long* d_zbuf, float* d_layer, int OW, int OH,
                  const float* d_verts, const float* d_cols, int ntris, CamDev v);
```

- [ ] **Step 5: Add upload_robot_mesh + composite hook in the Reprojector**

In `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/reprojector.hpp`, after `upload_depth`:

```cpp
    /** Upload the robot proxy mesh (persistent; call once). verts = n_tris*9
     *  rig-frame xyz, cols = n_tris*3 baked RGB (see mesh_loader.hpp).
     *  n_tris == 0 disables robot compositing. */
    void upload_robot_mesh(const float* verts, const float* cols, std::size_t n_tris);
```

In `cuda/src/libs/rendering_reprojector/src/reprojector.cpp`:

(a) In `struct Reprojector::Impl`, after the point-cloud device-buffer block:

```cpp
    // Robot proxy mesh (persistent, uploaded once; n_rtris == 0 -> no robot).
    float* d_rverts = nullptr;  size_t d_rverts_cap = 0;
    float* d_rcols  = nullptr;  size_t d_rcols_cap  = 0;
    int    n_rtris  = 0;
```

(b) In `Impl::free_all()`, before the closing brace:

```cpp
        if (d_rverts) cudaFree(d_rverts);
        if (d_rcols)  cudaFree(d_rcols);
```

(c) In `struct Reprojector::Impl`, after `sync_cams()`, add a member function:

```cpp
    // Rasterize the robot proxy and overlay it on the environment already in
    // d_out. Reuses d_zbuf + d_bowl as scratch (both free at this point in
    // every render path: bowl/depth don't use them afterwards, hybrid has
    // already consumed them into d_out). No-op without a mesh.
    void composite_robot(const CamDev& v)
    {
        if (n_rtris == 0) return;
        int npx = out_w * out_h;
        ensure_bytes(reinterpret_cast<void**>(&d_zbuf), d_zbuf_cap,
                     sizeof(unsigned long long) * npx);
        ensure_bytes(reinterpret_cast<void**>(&d_bowl), d_bowl_cap,
                     sizeof(float) * npx * 4);
        launch_robot(d_zbuf, d_bowl, out_w, out_h, d_rverts, d_rcols, n_rtris, v);
        // robot-where-valid else env: exactly launch_composite's contract
        launch_composite(d_bowl, d_out, d_out, npx);
    }
```

(d) After `upload_depth` (file scope, inside the namespace):

```cpp
void Reprojector::upload_robot_mesh(const float* verts, const float* cols, std::size_t n_tris)
{
    impl_->n_rtris = static_cast<int>(n_tris);
    if (n_tris == 0) return;
    size_t vb = n_tris * 9 * sizeof(float);
    size_t cb = n_tris * 3 * sizeof(float);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_rverts), impl_->d_rverts_cap, vb);
    impl_->ensure_bytes(reinterpret_cast<void**>(&impl_->d_rcols), impl_->d_rcols_cap, cb);
    CUDA_CHECK(cudaMemcpy(impl_->d_rverts, verts, vb, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(impl_->d_rcols, cols, cb, cudaMemcpyHostToDevice));
}
```

(e) Hook into all three render paths, immediately before the final
`cudaMemcpy(out_rgba, ...)` in each of `render_bowl`, `render_depth`, and
`render_hybrid` (after their existing `launch_*` / `CUDA_CHECK(cudaGetLastError())` lines):

```cpp
    impl_->composite_robot(v);
    CUDA_CHECK(cudaGetLastError());
```

(`render_bowl` and `render_depth` never allocate `d_bowl` today — `composite_robot`'s `ensure_bytes` covers that on first use.)

- [ ] **Step 6: Add the .cu to CMake and the binding**

`cuda/src/libs/rendering_reprojector/CMakeLists.txt`:

```cmake
add_library(rendering_reprojector SHARED
    src/reprojector.cpp
    src/mesh_loader.cpp
    src/kernels/fill.cu
    src/kernels/reproject.cu
    src/kernels/splat.cu
    src/kernels/robot_raster.cu
)
```

`bindings/tps_pybind.cpp`, inside the `py::class_<Reprojector>` chain (after `upload_depth`):

```cpp
        .def("upload_robot_mesh",
             [](Reprojector& r,
                py::array_t<float, py::array::c_style | py::array::forcecast> verts,
                py::array_t<float, py::array::c_style | py::array::forcecast> cols) {
                 if (verts.ndim() != 2 || verts.shape(1) != 9 || cols.ndim() != 2 ||
                     cols.shape(1) != 3 || verts.shape(0) != cols.shape(0))
                     throw std::invalid_argument(
                         "upload_robot_mesh expects verts (N,9) and cols (N,3)");
                 r.upload_robot_mesh(verts.data(), cols.data(),
                                     static_cast<std::size_t>(verts.shape(0)));
             })
```

- [ ] **Step 7: Build and run the tests**

Run: `cd cuda/scripts/libs_build && ./libs_build.sh Release && cd ../../..`
Then: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_robot_proxy.py -v`
Expected: 5 PASS.

- [ ] **Step 8: Full-suite regression**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest`
Expected: everything passes (notably `tests/test_cuda_parity.py` — no robot uploaded there, so outputs must be bit-identical to before).

- [ ] **Step 9: Commit**

```bash
git add cuda/src/libs/rendering_reprojector/src/kernels/robot_raster.cu \
        cuda/src/libs/rendering_reprojector/src/kernels/camdev.hpp \
        cuda/src/libs/rendering_reprojector/include/rendering_reprojector/reprojector.hpp \
        cuda/src/libs/rendering_reprojector/src/reprojector.cpp \
        cuda/src/libs/rendering_reprojector/CMakeLists.txt \
        cuda/src/libs/rendering_reprojector/bindings/tps_pybind.cpp \
        tests/test_robot_proxy.py
git commit -m "feat(cuda): robot proxy raster kernel + upload_robot_mesh, composited in all render paths"
```

---

### Task 3: Node wiring + config

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp` (params + load in `on_configure`)
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/config/default_params.yaml` (new params)
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/scripts/autotune_config.py` (passthrough so regeneration keeps the params)

**Interfaces:**
- Consumes: `micropilot::rendering::load_obj_mesh` (Task 1), `Reprojector::upload_robot_mesh` (Task 2).
- Produces: ROS params `robot_model_path` (string, default `""`), `robot_model_transform` (12 doubles, default `[1,0,0, 0,0,-1, 0,1,0, 0,0,0]`).

- [ ] **Step 1: Wire the node**

In `cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp`, add next to the existing includes:

```cpp
#include "rendering_reprojector/mesh_loader.hpp"
```

Then in `on_configure`, immediately after the "build reprojector" try/catch block (after `reprojector_->set_cameras(cam_params_);` succeeded, before the publishers section), insert:

```cpp
    // ── robot proxy mesh (optional) ──────────────────────────────────────────
    // No real camera sees the robot, so the virtual view composites a known 3D
    // proxy. robot_model_path = "" disables it; load failure is non-fatal
    // (render without the robot rather than take the node down).
    auto robot_path = declare_parameter<std::string>("robot_model_path", "");
    // OBJ->rig [R(9 row-major)|t(3)]. Default matches the M02P Blender export:
    // obj x-fwd, y-up, z-right -> rig x-fwd, y-left, z-up.
    auto robot_tf = declare_parameter<std::vector<double>>(
        "robot_model_transform", {1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0});
    if (!robot_path.empty())
    {
        if (robot_tf.size() != 12)
        {
            RCLCPP_ERROR(get_logger(),
                         "robot_model_transform must be 12 floats [R(9)|t(3)], got %zu",
                         robot_tf.size());
            return CallbackReturn::FAILURE;
        }
        float T[12];
        for (int i = 0; i < 12; ++i) T[i] = static_cast<float>(robot_tf[i]);
        try
        {
            auto mesh = micropilot::rendering::load_obj_mesh(robot_path, T);
            reprojector_->upload_robot_mesh(mesh.verts.data(), mesh.cols.data(),
                                            mesh.n_tris);
            RCLCPP_INFO(get_logger(), "robot proxy: %zu triangles from %s",
                        mesh.n_tris, robot_path.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(),
                         "robot model load failed, rendering without robot: %s", e.what());
        }
    }
```

- [ ] **Step 2: Add the params to default_params.yaml**

In `cuda/src/ros_apps/src/micropilot_rendering_node/config/default_params.yaml`, after the `feather_margin` line, add:

```yaml
    robot_model_path: /home/ag7/Downloads/M02P.obj
    robot_model_transform:
    - 1.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - -1.0
    - 0.0
    - 1.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
```

- [ ] **Step 3: Autotune passthrough**

In `cuda/src/ros_apps/src/micropilot_rendering_node/scripts/autotune_config.py`:

Add two arguments next to the other `ap.add_argument` lines:

```python
    ap.add_argument("--robot-model", default="/home/ag7/Downloads/M02P.obj",
                    help="robot proxy OBJ path written to robot_model_path ('' disables)")
    ap.add_argument("--robot-transform",
                    default="1,0,0,0,0,-1,0,1,0,0,0,0",
                    help="12 floats [R(9)|t(3)] OBJ->rig for robot_model_transform")
```

In the `params` dict (section `# 5. write config`), after the `"feather_margin"` entry, add:

```python
        "robot_model_path": a.robot_model,
        "robot_model_transform": [float(x) for x in a.robot_transform.split(",")],
```

- [ ] **Step 4: Build the node**

Run: `cd cuda/scripts/ros_apps_build && ./colcon_build.sh micropilot_rendering_node && cd ../../..`
Expected: build succeeds. (The install space symlinks config files, so `default_params.yaml` changes are live without re-copying.)

- [ ] **Step 5: Headless smoke test**

Run the existing node smoke test to prove configure/activate/render still work with the new params in the config (it loads the full M02P — expect a few extra seconds of configure time):
`source /opt/ros/humble/setup.bash && source cuda/install/ros_apps/setup.bash && python3 cuda/src/ros_apps/src/micropilot_rendering_node/test/smoke_test.py`
(Consult the smoke test's file header if it documents a different invocation.)
Expected: exit 0.

- [ ] **Step 6: Commit**

```bash
git add cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp \
        cuda/src/ros_apps/src/micropilot_rendering_node/config/default_params.yaml \
        cuda/src/ros_apps/src/micropilot_rendering_node/scripts/autotune_config.py
git commit -m "feat(ros): robot proxy params + mesh load/upload in rendering node"
```

---

### Task 4: Live visual verification + docs

**Files:**
- Modify: `README.md` (roadmap item 7, ROS2 LifecycleNode bullet)

**Interfaces:**
- Consumes: everything above, plus the running CARLA rig.
- Produces: visual confirmation (the user's acceptance gate) + one doc paragraph.

- [ ] **Step 1: Verify on the live rig**

With CARLA + the camera bridge running (ask the user to start them if they are not):

```bash
# restart the rendering node so on_configure reloads params & the mesh
source /opt/ros/humble/setup.bash && source cuda/install/ros_apps/setup.bash
ros2 launch micropilot_rendering_node rendering_node.launch.py
# then: python3 tools/vcam_ws_bridge.py  and  python3 tools/vcam_gui.py
```

Check in the GUI across presets top_down / left_side / config:
- robot body visible at the frame center (white body, black wheels, sirens);
- headlights face the driving direction — if the robot renders backwards, flip
  the transform yaw in `default_params.yaml`:
  `robot_model_transform: [-1,0,0, 0,0,1, 0,1,0, 0,0,0]`;
- wheels touch the ground (not floating or sunk);
- startup log shows `robot proxy: ~1,00x,xxx triangles from /home/ag7/Downloads/M02P.obj`;
- frame rate unchanged vs. before (raster is ~1 ms-scale on the 3090).

Take a screenshot for the user to judge — their acceptance gate is visual.

- [ ] **Step 2: README line**

In `README.md`, roadmap item 7's ROS2 LifecycleNode bullet, after the sentence about runtime virtual-cam control, add:

```
     Robot proxy: `robot_model_path` (OBJ+MTL, e.g. the M02P model) is loaded at
     configure time, rasterized on the GPU, and depth-composited into every
     rendered frame — the robot's own body is visible in the virtual view even
     though no real camera sees it. `robot_model_transform` ([R|t], default
     Blender-export → rig) is the model-orientation calibration knob.
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs: robot proxy rendering in README roadmap"
```

---

## Self-Review (done at plan-writing time)

- **Spec coverage:** loader+MTL+gray-fallback (Task 1), transform config knob (Tasks 1/3), upload-once + raster kernel + composite in all three render modes (Task 2), `robot_model_path` config + non-fatal error handling + autotune passthrough (Task 3), model outside git (Global Constraints), loader/triangle/depth/visual tests (Tasks 1/2/4). Spec's "loader node-side" moved to the lib — declared as a deviation in the header with rationale.
- **Placeholder scan:** none — every step carries complete code/commands.
- **Type consistency:** `load_obj_mesh(path, const float*)` / `RobotMesh{verts, cols, n_tris}` / `upload_robot_mesh(const float*, const float*, std::size_t)` / `launch_robot(zbuf, layer, OW, OH, verts, cols, ntris, v)` used identically across Tasks 1–3; Python binding signatures match.
