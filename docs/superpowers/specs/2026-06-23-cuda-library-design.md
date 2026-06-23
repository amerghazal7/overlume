# C++/CUDA TPSProjector Library + ROS2 Integration — Design Spec

**Date:** 2026-06-23
**Status:** Approved design, pre-implementation
**Author:** brainstorming session
**Builds on:** the validated Python/GL prototype (`Renderer`/`Surface` seams) and
roadmap item "C++/CUDA library — port the validated design; clean C++ API
consumable by other projects."

## 1. Summary

Port the validated TPSProjector reprojection (bowl, depth, hybrid) to a
standalone **CUDA C++ library** that follows the **micropilot_manager** code and
CMake conventions, so it drops into the micropilot ROS2 ecosystem as
`micropilot_rendering`. The library is **ROS-agnostic**; a thin
`rclcpp_lifecycle::LifecycleNode` wraps it and is the real integration test.
Correctness is gated by **pybind11 parity against the existing Python reference**
(`NumpyRenderer`/`DepthRenderer`), plus a native C++ golden test and a ROS2 launch
smoke test. Built inside the TPSProjector repo under `cuda/`, mirroring
micropilot's two-tier layout, so it is a clean copy/move into `~/micropilot` later.

## 2. Goals & Non-Goals

### Goals
- A pure C++/CUDA reprojection library (`micropilot::rendering`) with bowl, depth,
  and hybrid rendering, numerically matching the validated NumPy/GL paths.
- **Headless-first:** the library computes the reprojected TPS frame on the GPU with
  **no GL, no windowing, and no ROS dependency** (links only `CUDA::cudart`); it returns
  a frame buffer. The same output feeds any consumer — see "Consumer modes" below.
- A clean public C++ API consumable via `find_package(micropilot_rendering)` →
  `micropilot_rendering::reprojector` (manager-style CMake package export).
- A `pybind11` module so the existing Python test suite validates the CUDA output
  (pixel-parity vs `NumpyRenderer`/`DepthRenderer`).
- A thin `LifecycleNode` ROS2 wrapper (ament_cmake) that synthesizes a virtual
  camera view from real camera topics — the real integration test.
- Match micropilot_manager conventions exactly (namespace, headers, CMake, style).

### Non-Goals (YAGNI)
- Device-pointer / zero-copy GL-CUDA interop API (designed-for extension, not v1).
- Texture-object hardware bilinear (manual in-kernel bilinear first, for exact parity).
- A real depth source — the ROS2 node's depth/hybrid path awaits perception's
  LiDAR-projection depth; the node's integration test uses **bowl** mode.
- Moving the package into `~/micropilot` (kept drop-in-ready in TPSProjector).
- Multi-GPU, FP16.

## 3. Environment (verified on this box)

- CUDA 12.4 (`nvcc`; also 12.8 present); RTX 3090, compute capability **8.6** → `sm_86`.
- `g++` 11.4, CMake 3.28, make. ROS2 **Humble** (`/opt/ros/humble`, `ros2`, `colcon`, `rclcpp`).
- `pybind11` to be installed (`pip install pybind11`, provides its CMake config).
- micropilot's CUDA libs target arch 89 for their deploy GPU; we use `sm_86` for this
  box, exposed as a CMake cache var `CMAKE_CUDA_ARCHITECTURES` so it is retargetable.

## 4. Conventions (micropilot_manager style — strictly)

- **Namespace:** `micropilot::rendering` (manager uses `micropilot::<concern>::<module>`,
  e.g. `micropilot::system_manager::core`). ROS node: `micropilot::rendering_app`.
- **Headers:** `.hpp`, `#pragma once`, Doxygen `/** @file @brief @class */` comments.
- **C++ standard:** library **C++14** (manager rule); ROS node **C++17**. CUDA standard 14.
- **Style:** Google-based clang-format — IndentWidth 4, ColumnLimit 100, Allman braces,
  SortIncludes. Classes `PascalCase`; methods/functions `snake_case`; members
  `snake_case_`; enum-class values `UPPER_SNAKE_CASE`.
- **Tests:** GoogleTest, aggregated into one `test_*` executable with
  `gtest_discover_tests`, under a `BUILD_TESTING` option (manager pattern).
- **Build:** plain CMake for the lib (manager-style package export); ament_cmake/colcon
  for the ROS node. **Build scripts mirror manager's exactly:**
  `scripts/libs_build/libs_build.sh <Debug|Release>` (Debug ⇒ `-DBUILD_TESTING=ON`; creates
  `build/`; `cmake -DCMAKE_BUILD_TYPE=… ..`; `cmake --build . --parallel $(($(nproc)/2))`;
  `cmake --install .`) and `scripts/ros_apps_build/colcon_build.sh [pkgs]` (`cd src/ros_apps`;
  `COLCON_DEFAULTS_FILE=config_colcon.yaml`; `colcon build --symlink-install
  --parallel-workers $(($(nproc)/2)) --event-handlers console_direct+ --cmake-args
  -DCMAKE_BUILD_TYPE=… -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`). Tests build only in Debug
  (manager rule: root CMake gates `test_*` on `Debug` + `BUILD_TESTING`). Install to
  `install/libs/` and `install/ros_apps/`.

## 5. Repository layout (inside TPSProjector, mirrors micropilot two-tier)

```
cuda/
  CMakeLists.txt                                  # root: C++14, CUDA, package export
  cmake/micropilot_renderingConfig.cmake.in       # find_dependency(CUDAToolkit), targets
  scripts/libs_build/libs_build.sh                # <Debug|Release>; cmake --build/--install -> install/libs/
  scripts/ros_apps_build/colcon_build.sh          # [pkgs]; colcon --symlink-install -> install/ros_apps/
  scripts/ros_apps_build/config_colcon.yaml       # colcon defaults (mirrors manager)
  src/libs/CMakeLists.txt                         # add_subdirectory(rendering_reprojector)
  src/libs/rendering_reprojector/
    include/rendering_reprojector/                # public headers (micropilot::rendering)
      types.hpp          # CameraParams, BowlParams
      reprojector.hpp    # class Reprojector
    src/
      reprojector.cpp    # host orchestration + device buffer management
      kernels/reproject.cu, splat.cu
      kernels/surface.cuh, sampling.cuh, blend.cuh   # __device__ helpers
    CMakeLists.txt       # add_library SHARED; EXPORT_NAME reprojector; CUDA arch var
  src/libs/rendering_reprojector/bindings/
    tps_pybind.cpp       # pybind11 module `tpscuda`
    CMakeLists.txt
  src/libs/rendering_reprojector/tests/
    test_reprojector.cpp # GTest golden parity (C++)
  examples/render_demo.cpp                        # standalone no-Python consumer
  src/ros_apps/src/micropilot_rendering_node/     # thin LifecycleNode (ament_cmake, C++17)
    include/micropilot_rendering_node/rendering_node.hpp
    src/rendering_node.cpp, main.cpp
    launch/rendering_node.launch.py
    package.xml, CMakeLists.txt
tests/                                            # (existing Python tests unchanged)
```

## 6. Pure C++/CUDA library (`micropilot::rendering`)

### 6.1 Public API
```cpp
namespace micropilot::rendering {
struct CameraParams { float K[9]; float R[9]; float t[3]; int width; int height; };  // R cols = right,down,fwd
struct BowlParams   { float R0; float k; float Rmax; };

class Reprojector {
public:
    Reprojector(int out_width, int out_height);
    ~Reprojector();
    void set_cameras(const std::vector<CameraParams>& cams);            // calibrations (once)
    void upload_images(const float* nhwc, int n, int h, int w);         // (N,H,W,3) (once for static scene)
    void upload_depth(const float* nhw, int n, int h, int w);           // optional (depth/hybrid)
    void render_bowl  (const CameraParams& v, const BowlParams& b, float* out_rgba);
    void render_depth (const CameraParams& v, int splat_radius, float* out_rgba);
    void render_hybrid(const CameraParams& v, const BowlParams& b, int splat_radius, float* out_rgba);
};
}  // namespace micropilot::rendering
```
- `out_rgba` is host `(H, W, 4)` row-major float (row 0 = top), matching the Python
  frame orientation; RGB = color, A = valid mask (>0.5 means a camera/point hit).
- `set_cameras` / `upload_images` split bakes in the upload-once lesson.
- Host arrays in/out in v1; device-pointer overloads are a designed-for extension.

### 6.3 Consumer modes (the library depends on none of them)
The library is headless by construction; the returned host RGBA frame is consumed by:
1. **ROS app** — the `LifecycleNode` (§9) converts the frame via `cv_bridge` and publishes
   `sensor_msgs/Image`. No GL/window.
2. **Pure C++ executable** — render frames and write them to a video (e.g. OpenCV
   `VideoWriter`) or image files. No GL/window. (Demonstrated by the standalone example.)
3. **C++ GL windowed app** — a GL consumer uploads the frame as a texture and blits it to a
   window. The core lib stays GL-free; an optional **device-pointer output overload** (CUDA→GL
   interop, zero host copy) is a designed-for extension for this consumer's efficiency, not v1.

None of these are required to build/test the core library — the lib + pybind + GTest build and
validate with no ROS, no GL, and no display.

### 6.2 Kernels & numeric parity (same math as the validated paths)
- **Bowl** (`reproject.cu`): one thread per output pixel; reconstruct the world ray
  from the virtual `CameraParams`; intersect the surface (`surface.cuh`): flat plane
  closed-form, or bowl bisection `height(r)=k*clamp(r-R0,0,Rmax-R0)^2`, `g(t)=Pz-height`,
  bracket `[1e-6,1e4]`, **60 iterations**, valid iff `g(eps)>0 && g(tmax)<0`; reproject
  into each camera (`R^T(P-t)`, `z>1e-9`, in-bounds); **manual bilinear with edge clamp**
  (`sampling.cuh`, exact `NumpyRenderer.bilinear_sample` match); weight
  `feather*align^2` (`blend.cuh`); normalized accumulate; A=1 where `wsum>0`.
- **Depth** (`splat.cu`): back-project each finite-depth source pixel to a world point
  on the host (matching `DepthRenderer._point_cloud`), upload once; a kernel projects
  points into the virtual camera, splats a `(2r+1)` square, and resolves nearest-wins
  in a single pass via `atomicMin` on a per-pixel `uint64` packed as
  `(float_bits(z) << 32) | point_index` — for positive `z`, IEEE float bit patterns are
  monotonic, so the smallest packed value is the nearest point and its low 32 bits index
  the winning color. A cheap second kernel scatters winners' colors to the output.
- **Hybrid:** depth where valid else bowl, composited on device (same rule as Python).

## 7. CMake (manager-style package export + CUDA)

Root `cuda/CMakeLists.txt` mirrors `micropilot_manager/CMakeLists.txt`:
`cmake_minimum_required(3.20)`; `CMAKE_CXX_STANDARD 14`, PIC on, exports on;
`project(micropilot_rendering VERSION 0.1.0 LANGUAGES CXX CUDA)`;
`set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "")`; `CMAKE_CUDA_STANDARD 14`;
install prefix `${CMAKE_SOURCE_DIR}/install/libs`, per-lib RPATH; `add_subdirectory(src/libs)`;
then the package export block:
```cmake
install(EXPORT micropilot_renderingTargets NAMESPACE micropilot_rendering::
        DESTINATION lib/cmake/micropilot_rendering)
configure_package_config_file(cmake/micropilot_renderingConfig.cmake.in ...)
write_basic_package_version_file(... COMPATIBILITY AnyNewerVersion)
install(FILES <Config.cmake> <ConfigVersion.cmake> DESTINATION lib/cmake/micropilot_rendering)
```
Each lib `CMakeLists.txt` mirrors `micropilot_manager_core`:
`add_library(rendering_reprojector SHARED ...)`,
`set_target_properties(... EXPORT_NAME reprojector)`,
`target_include_directories(... BUILD_INTERFACE/INSTALL_INTERFACE ...)`,
`install(TARGETS ... EXPORT micropilot_renderingTargets LIBRARY DESTINATION rendering_reprojector/libs)`,
`install(DIRECTORY include/ DESTINATION rendering_reprojector/include/rendering_reprojector)`.
The CUDA toolkit is found via `find_package(CUDAToolkit REQUIRED)` (cleaner than the
perception manual-path approach) and linked as `CUDA::cudart`.
`Config.cmake.in` does `find_dependency(CUDAToolkit)` then includes the targets file.
Consumers: `find_package(micropilot_rendering REQUIRED)` → link `micropilot_rendering::reprojector`.

## 8. pybind11 module + parity validation (primary correctness gate)

`tps_pybind.cpp` exposes `Reprojector` as the `tpscuda` Python module (NumPy arrays
in/out via the buffer protocol). New Python tests under TPSProjector `tests/` reuse the
existing scene/rig fixtures and the validated reference renderers:
- `tpscuda` bowl vs `NumpyRenderer` → mask agreement > 0.97 AND **PSNR > 40 dB**.
- `tpscuda` depth vs `DepthRenderer` → mask > 0.90 AND PSNR > 28 dB.
- `tpscuda` hybrid vs the Python hybrid composite → PSNR > 28 dB.
All skip-guarded with a `cuda_available()` probe so the suite still passes without a GPU.

## 9. ROS2 LifecycleNode (thin wrapper — the integration test)

`micropilot_rendering_node` (ament_cmake, C++17, namespace `micropilot::rendering_app`):
- `rclcpp_lifecycle::LifecycleNode`; `on_configure` builds the `Reprojector`,
  `on_activate` starts processing, etc. (manager/perception lifecycle pattern).
- Subscribes N camera `sensor_msgs/Image` + `sensor_msgs/CameraInfo` (intrinsics →
  `CameraParams.K`); camera **extrinsics from tf2** (camera frame → rig frame), like
  `mp::perception::Transformations`. `cv_bridge` for image conversion.
- Virtual-camera pose from a parameter (named presets / pose offset in the rig frame).
- Publishes the synthesized view as `sensor_msgs/Image` + `CameraInfo`; reports
  `component_health` like the perception nodes.
- **Integration test:** a `launch.py`-based smoke test publishes synthetic Image+CameraInfo
  on N topics with known calibration, runs the node in **bowl** mode, and asserts it
  publishes a synthesized frame whose coverage/content is sane (non-blank). Depth/hybrid
  node paths await a real depth source (designed-for).

## 10. Testing & verification

1. **pybind11 parity** (§8) — primary, reuses validated Python refs.
2. **C++ GTest golden** — Python exports a few reference frames (raw f32 + a small shape
   sidecar); `test_reprojector.cpp` renders the same inputs and asserts PSNR in-process —
   proves the no-Python path and the `find_package` consumability via the example.
3. **ROS2 launch smoke test** (§9) — the real integration test.
4. **Benchmark** — a CUDA fps timer at 720p bowl, compared to the GL 882 fps number.
5. The existing **107 Python tests stay green** (additive `cuda/` tree; no edits to the
   Python package).

## 11. Build & run commands

- Library + pybind + GTest: `cd cuda/scripts/libs_build && ./libs_build.sh Debug`
  (manager script: Debug ⇒ `-DBUILD_TESTING=ON`, builds + `ctest` + installs to `install/libs/`).
- pybind parity (after build, module on `PYTHONPATH`):
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_cuda_parity.py`.
- ROS node: `cd cuda/scripts/ros_apps_build && ./colcon_build.sh micropilot_rendering_node`
  then the launch smoke test.

## 12. Build order (within the one plan)

1. Library scaffold + manager-style CMake package + `Reprojector` skeleton + CUDA arch.
2. Bowl kernel (surface/sampling/blend device helpers) + host buffer mgmt.
3. pybind11 module + bowl parity vs `NumpyRenderer` (>40 dB) — the first real proof.
4. Depth splat kernel + depth parity vs `DepthRenderer`.
5. Hybrid composite + hybrid parity.
6. C++ GTest golden test + standalone `render_demo` example + `find_package` consumability.
7. CMake package install/Config + benchmark.
8. ROS2 `LifecycleNode` + launch smoke integration test + README.

## 13. Risks & mitigations

- **Numeric parity drift (float32 vs float64, atomics ordering)** → 40/28 dB tolerances
  (not exact equality); match bisection iters, edge-clamp bilinear, and nearest-wins exactly.
- **pybind11 / CUDA toolchain availability** → `pip install pybind11`; `cuda_available()`
  skip-guard so the Python suite passes without a GPU.
- **Depth nearest-wins on GPU** → packed `atomicMin` z-buffer; validated against the
  NumPy `np.minimum.at` result by parity test (tolerant at splat edges).
- **ROS2 build coupling** → the node is a separate ament package; the core lib never
  depends on ROS, so the lib + pybind + GTest build and validate without ROS sourced.
- **tf2 extrinsics in the node** → for the smoke test, publish static transforms so the
  node has a deterministic rig; real deployment uses the live tf tree.

## 14. Out of scope (future slices)
Device-pointer / GL-CUDA zero-copy interop; texture-object hardware bilinear; FP16;
real depth source wiring for the node's depth/hybrid path; moving the package into
`~/micropilot`. Seams (`Reprojector` API, surface dispatch) keep these open.
