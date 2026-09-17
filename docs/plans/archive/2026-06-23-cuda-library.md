# C++/CUDA TPSProjector Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A standalone CUDA C++ reprojection library (bowl/depth/hybrid) following micropilot_manager conventions, validated by pybind11 parity against the Python reference, with a C++ golden test and a thin ROS2 LifecycleNode integration test.

**Architecture:** Pure C++/CUDA library `micropilot::rendering` (the validated NumPy/GL math on CUDA) under `cuda/`, mirroring micropilot's two-tier layout. A `pybind11` module exposes it so the existing Python parity harness validates it. A separate ament `LifecycleNode` wraps it. Manager-style CMake package export (`find_package(micropilot_rendering)`).

**Tech Stack:** CUDA 12.4 (`sm_86`), C++14 (lib) / C++17 (node), CMake 3.20, pybind11, ROS2 Humble, GoogleTest, pytest.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-06-23-cuda-library-design.md`.
- **Conventions (micropilot_manager, strict):** namespace `micropilot::rendering` (lib) / `micropilot::rendering_app` (node); headers `.hpp` + `#pragma once` + Doxygen; C++14 lib / C++17 node / CUDA std 14; Google clang-format (IndentWidth 4, ColumnLimit 100, Allman braces); classes `PascalCase`, methods `snake_case`, members `snake_case_`, enum-class `UPPER_SNAKE_CASE`; GoogleTest aggregated under `BUILD_TESTING` (Debug only).
- **CMake:** manager-style package export — `install(EXPORT micropilot_renderingTargets NAMESPACE micropilot_rendering:: DESTINATION lib/cmake/micropilot_rendering)`, `configure_package_config_file` + `write_basic_package_version_file`, per-lib `set_target_properties(... EXPORT_NAME <short>)`, `target_include_directories` with `BUILD_INTERFACE`/`INSTALL_INTERFACE`, install to `install/libs/<lib>/{libs,include/<lib>}`. CUDA via `find_package(CUDAToolkit REQUIRED)` + link `CUDA::cudart`. Arch: `set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "")`.
- **Build scripts mirror manager:** `cuda/scripts/libs_build/libs_build.sh <Debug|Release>` (Debug ⇒ `-DBUILD_TESTING=ON`; `build/` dir; `cmake --build . --parallel $(($(nproc)/2))`; `cmake --install .`); `cuda/scripts/ros_apps_build/colcon_build.sh [pkgs]` (+ `config_colcon.yaml`, `--symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=… -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`).
- **Numeric parity targets** (vs the validated Python reference, skip-guarded by `cuda_available()`): bowl vs `NumpyRenderer` mask>0.97 & **PSNR>40 dB**; depth vs `DepthRenderer` mask>0.90 & PSNR>28 dB; hybrid vs Python hybrid PSNR>28 dB.
- **Orientation:** CUDA writes output row 0 = top directly (no flip — unlike GL). `out_rgba` is host `(H,W,4)` float32 row-major; RGB=color, A=valid (1.0 hit / 0.0 miss).
- **Camera mapping:** `CameraParams{K[9],R[9],t[3],width,height}`; `K` row-major (fx=K[0],fy=K[4],cx=K[2],cy=K[5]); `R` row-major, columns = (right,down,fwd) so right=(R[0],R[3],R[6]), down=(R[1],R[4],R[7]), fwd=(R[2],R[5],R[8]); `t` = camera center. Same as Python `Pose`.
- **Headless-first (hard constraint):** the core library (`rendering_reprojector`) links ONLY `CUDA::cudart` — NO GL, no windowing, no ROS, no display. It returns a host RGBA frame. The lib + pybind + GTest must build and pass with no ROS sourced and no display. The three consumers (ROS publish, video/file write, GL window) depend on the lib, never the reverse.
- The existing **107 Python tests stay green** — this is an additive new `cuda/` tree; do NOT edit the `tpsprojector/` Python package.
- Test runner for Python parity: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest`. Use `python3`.

---

### Task 1: Library scaffold + manager-style CMake package + `Reprojector` skeleton

**Files:**
- Create: `cuda/CMakeLists.txt`, `cuda/cmake/micropilot_renderingConfig.cmake.in`
- Create: `cuda/scripts/libs_build/libs_build.sh`
- Create: `cuda/src/libs/CMakeLists.txt`
- Create: `cuda/src/libs/rendering_reprojector/CMakeLists.txt`
- Create: `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/{types.hpp,reprojector.hpp}`
- Create: `cuda/src/libs/rendering_reprojector/src/reprojector.cpp`
- Create: `cuda/src/libs/rendering_reprojector/tests/test_reprojector.cpp`
- Create: `cuda/.clang-format`

**Interfaces:**
- Produces: `micropilot::rendering::CameraParams`, `BowlParams`, and `class Reprojector` with the full public method set (stubbed render methods that zero the output for now). Target `rendering_reprojector` (EXPORT_NAME `reprojector`), package `micropilot_rendering`.

- [ ] **Step 1: Install pybind11 + confirm toolchain**

Run: `python3 -m pip install --user pybind11` then
`python3 -c "import pybind11; print(pybind11.get_cmake_dir())"`
Expected: prints a cmake dir (used later by the binding). CUDAToolkit is already findable (`find_package(CUDAToolkit)` resolves 12.4).

- [ ] **Step 2: Write `.clang-format`**

Create `cuda/.clang-format`:
```yaml
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 100
BreakBeforeBraces: Allman
SortIncludes: true
```

- [ ] **Step 3: Write the public headers**

Create `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/types.hpp`:
```cpp
#pragma once
/** @file types.hpp @brief Plain-old-data parameter structs for the reprojector. */

namespace micropilot::rendering
{
/** @brief Pinhole camera: row-major K and R, center t. R columns = (right,down,fwd). */
struct CameraParams
{
    float K[9];
    float R[9];
    float t[3];
    int width;
    int height;
};

/** @brief Bowl proxy surface: flat floor radius R0, parabolic wall k, clamp Rmax. */
struct BowlParams
{
    float R0;
    float k;
    float Rmax;
};
}  // namespace micropilot::rendering
```
Create `cuda/src/libs/rendering_reprojector/include/rendering_reprojector/reprojector.hpp`:
```cpp
#pragma once
/** @file reprojector.hpp @brief CUDA virtual-camera reprojection (bowl/depth/hybrid). */

#include <cstddef>
#include <vector>

#include "rendering_reprojector/types.hpp"

namespace micropilot::rendering
{
/**
 * @class Reprojector
 * @brief Synthesizes a virtual camera view by reprojecting N real cameras on the GPU.
 *
 * Upload calibrations and images once (static scene), then render per virtual pose.
 * Output is host (H,W,4) float row-major, row 0 = top; RGB = color, A = valid mask.
 */
class Reprojector
{
public:
    Reprojector(int out_width, int out_height);
    ~Reprojector();
    Reprojector(const Reprojector&) = delete;
    Reprojector& operator=(const Reprojector&) = delete;

    void set_cameras(const std::vector<CameraParams>& cams);
    void upload_images(const float* nhwc, int n, int h, int w);
    void upload_depth(const float* nhw, int n, int h, int w);

    void render_bowl(const CameraParams& vcam, const BowlParams& bowl, float* out_rgba);
    void render_depth(const CameraParams& vcam, int splat_radius, float* out_rgba);
    void render_hybrid(const CameraParams& vcam, const BowlParams& bowl, int splat_radius,
                       float* out_rgba);

private:
    struct Impl;
    Impl* impl_;
};
}  // namespace micropilot::rendering
```

- [ ] **Step 4: Write the skeleton implementation**

Create `cuda/src/libs/rendering_reprojector/src/reprojector.cpp`:
```cpp
/** @file reprojector.cpp @brief Host orchestration + device buffer management (skeleton). */

#include "rendering_reprojector/reprojector.hpp"

#include <cstring>
#include <vector>

namespace micropilot::rendering
{
struct Reprojector::Impl
{
    int out_w;
    int out_h;
    std::vector<CameraParams> cams;
    int img_n = 0, img_h = 0, img_w = 0;
};

Reprojector::Reprojector(int out_width, int out_height) : impl_(new Impl)
{
    impl_->out_w = out_width;
    impl_->out_h = out_height;
}

Reprojector::~Reprojector() { delete impl_; }

void Reprojector::set_cameras(const std::vector<CameraParams>& cams) { impl_->cams = cams; }

void Reprojector::upload_images(const float*, int n, int h, int w)
{
    impl_->img_n = n;
    impl_->img_h = h;
    impl_->img_w = w;
}

void Reprojector::upload_depth(const float*, int, int, int) {}

void Reprojector::render_bowl(const CameraParams&, const BowlParams&, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}

void Reprojector::render_depth(const CameraParams&, int, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}

void Reprojector::render_hybrid(const CameraParams&, const BowlParams&, int, float* out_rgba)
{
    std::memset(out_rgba, 0, sizeof(float) * impl_->out_w * impl_->out_h * 4);
}
}  // namespace micropilot::rendering
```

- [ ] **Step 5: Write the library CMake (manager pattern)**

Create `cuda/src/libs/rendering_reprojector/CMakeLists.txt`:
```cmake
message("############# rendering_reprojector Lib CMake ##################")

find_package(CUDAToolkit REQUIRED)

add_library(rendering_reprojector SHARED
    src/reprojector.cpp
)
set_target_properties(rendering_reprojector PROPERTIES EXPORT_NAME reprojector)

target_include_directories(rendering_reprojector
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:rendering_reprojector/include/rendering_reprojector>
)
target_link_libraries(rendering_reprojector PUBLIC CUDA::cudart)

install(TARGETS rendering_reprojector
        EXPORT micropilot_renderingTargets
        LIBRARY DESTINATION rendering_reprojector/libs)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION rendering_reprojector/include/rendering_reprojector)

if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND)
        add_executable(test_reprojector tests/test_reprojector.cpp)
        target_link_libraries(test_reprojector PRIVATE rendering_reprojector GTest::GTest GTest::Main)
        include(GoogleTest)
        gtest_discover_tests(test_reprojector DISCOVERY_TIMEOUT 30)
    else()
        message(STATUS "GTest not found; skipping rendering_reprojector tests.")
    endif()
endif()
```
Create `cuda/src/libs/CMakeLists.txt`:
```cmake
message("############# Micropilot Rendering Libs CMake ##################")
add_subdirectory(rendering_reprojector)
```

- [ ] **Step 6: Write the root CMake (package export, CUDA)**

Create `cuda/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME "micropilot_rendering")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_ENABLE_EXPORTS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CUDA_ARCHITECTURES 86 CACHE STRING "CUDA arch (86=RTX3090; set 89 for Ada deploy)")

set(MPR_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install/libs")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${MPR_INSTALL_PREFIX}/rendering_reprojector/libs")

project("${PROJECT_NAME}" VERSION 0.1.0 DESCRIPTION "micropilot_rendering" LANGUAGES CXX CUDA)

set(CMAKE_INSTALL_PREFIX "${MPR_INSTALL_PREFIX}" CACHE PATH "" FORCE)
option(BUILD_TESTING "Build unit tests" OFF)
option(BUILD_PYBIND "Build the tpscuda pybind11 module" ON)

add_subdirectory(src/libs)

include(CMakePackageConfigHelpers)
install(EXPORT micropilot_renderingTargets
    NAMESPACE micropilot_rendering::
    DESTINATION lib/cmake/micropilot_rendering)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/micropilot_renderingConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfig.cmake"
    INSTALL_DESTINATION lib/cmake/micropilot_rendering)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/micropilot_renderingConfigVersion.cmake"
    DESTINATION lib/cmake/micropilot_rendering)
```
Create `cuda/cmake/micropilot_renderingConfig.cmake.in`:
```cmake
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(CUDAToolkit)
include("${CMAKE_CURRENT_LIST_DIR}/micropilot_renderingTargets.cmake")
check_required_components(micropilot_rendering)
```

- [ ] **Step 7: Write the build script (manager pattern)**

Create `cuda/scripts/libs_build/libs_build.sh` (mark executable):
```bash
#!/bin/bash
# $1 = build type (Debug|Release). Debug enables tests.
BUILD_TYPE=$1
if [ -z "$BUILD_TYPE" ]; then
    echo "Usage: $0 <Debug|Release>"; exit 1
fi
if [ "$BUILD_TYPE" == "Debug" ]; then
    BUILD_ARGS="-DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON"
elif [ "$BUILD_TYPE" == "Release" ]; then
    BUILD_ARGS="-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
else
    echo "Unknown build type '$BUILD_TYPE'."; exit 1
fi
cd ../../
BUILD_DIR="build"
[ -d "$BUILD_DIR" ] || mkdir "$BUILD_DIR"
cd $BUILD_DIR
cmake $BUILD_ARGS ..
cmake --build . --parallel $(($(nproc)/2))
cmake --install .
```
Then: `chmod +x cuda/scripts/libs_build/libs_build.sh`.

- [ ] **Step 8: Write the skeleton GTest**

Create `cuda/src/libs/rendering_reprojector/tests/test_reprojector.cpp`:
```cpp
#include <gtest/gtest.h>

#include <vector>

#include "rendering_reprojector/reprojector.hpp"

using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

TEST(Reprojector, ConstructsAndRendersBufferOfCorrectSize)
{
    Reprojector r(8, 6);
    std::vector<float> out(8 * 6 * 4, -1.0f);
    BowlParams bowl{6.0f, 0.08f, 20.0f};
    CameraParams v{};
    v.width = 8;
    v.height = 6;
    r.render_bowl(v, bowl, out.data());
    // skeleton zeroes the buffer; just assert it wrote all elements
    for (float val : out) EXPECT_EQ(val, 0.0f);
}
```

- [ ] **Step 9: Build + test**

Run: `cd cuda/scripts/libs_build && ./libs_build.sh Debug`
Expected: configures (CUDA enabled, CUDAToolkit found), builds `librendering_reprojector.so`, runs `test_reprojector` (1 test passes), installs to `cuda/install/libs/rendering_reprojector/`. Confirm `cuda/install/libs/lib/cmake/micropilot_rendering/micropilot_renderingConfig.cmake` exists.

- [ ] **Step 10: Commit**

```bash
cd /home/ag7/Documents/TPSProjector
echo "cuda/build/" >> .gitignore; echo "cuda/install/" >> .gitignore
git add cuda/ .gitignore
git commit -m "feat(cuda): library scaffold — manager-style CMake package + Reprojector skeleton"
```

---

### Task 2: pybind11 module plumbing + toolchain round-trip

Prove CUDA → pybind11 → NumPy end-to-end with a trivial device kernel before the real math.

**Files:**
- Create: `cuda/src/libs/rendering_reprojector/bindings/{tps_pybind.cpp,CMakeLists.txt}`
- Create: `cuda/src/libs/rendering_reprojector/src/kernels/fill.cu`
- Modify: `cuda/src/libs/rendering_reprojector/CMakeLists.txt` (compile the `.cu`), `cuda/src/libs/CMakeLists.txt` (add bindings)
- Modify: `reprojector.cpp` (call the fill kernel from `render_bowl` temporarily — replaced in Task 3)
- Test: `tests/test_cuda_parity.py` (new, in the repo `tests/` dir) — round-trip check

**Interfaces:**
- Produces: Python module `tpscuda` exposing `Reprojector(out_w,out_h)` with `set_cameras(list-of-dicts or arrays)`, `upload_images(ndarray NHWC)`, `render_bowl(vcam, bowl)->ndarray (H,W,4)`. A `cuda_available()` helper in the test.

- [ ] **Step 1: Write the trivial CUDA kernel**

Create `cuda/src/libs/rendering_reprojector/src/kernels/fill.cu`:
```cpp
/** @file fill.cu @brief Toolchain smoke kernel: write a constant RGBA per pixel. */
#include <cuda_runtime.h>

namespace micropilot::rendering
{
__global__ void fill_kernel(float* out, int n, float r, float g, float b, float a)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    out[i * 4 + 0] = r;
    out[i * 4 + 1] = g;
    out[i * 4 + 2] = b;
    out[i * 4 + 3] = a;
}

void launch_fill(float* d_out, int n, float r, float g, float b, float a)
{
    int threads = 256;
    fill_kernel<<<(n + threads - 1) / threads, threads>>>(d_out, n, r, g, b, a);
}
}  // namespace micropilot::rendering
```
In `reprojector.cpp`, declare `void launch_fill(float*,int,float,float,float,float);` in the namespace, allocate a device buffer in `render_bowl`, call `launch_fill` with `(0.1,0.2,0.3,1.0)`, `cudaMemcpy` back to `out_rgba`. (This is temporary scaffolding; Task 3 replaces it with the real kernel.) Add `#include <cuda_runtime.h>`.

- [ ] **Step 2: Write the pybind module**

Create `cuda/src/libs/rendering_reprojector/bindings/tps_pybind.cpp`:
```cpp
/** @file tps_pybind.cpp @brief Python bindings (`tpscuda`) for parity testing. */
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "rendering_reprojector/reprojector.hpp"

namespace py = pybind11;
using micropilot::rendering::BowlParams;
using micropilot::rendering::CameraParams;
using micropilot::rendering::Reprojector;

static CameraParams to_cam(py::dict d)
{
    CameraParams c{};
    auto K = d["K"].cast<py::array_t<float>>();
    auto R = d["R"].cast<py::array_t<float>>();
    auto t = d["t"].cast<py::array_t<float>>();
    std::memcpy(c.K, K.data(), 9 * sizeof(float));
    std::memcpy(c.R, R.data(), 9 * sizeof(float));
    std::memcpy(c.t, t.data(), 3 * sizeof(float));
    c.width = d["width"].cast<int>();
    c.height = d["height"].cast<int>();
    return c;
}

PYBIND11_MODULE(tpscuda, m)
{
    py::class_<Reprojector>(m, "Reprojector")
        .def(py::init<int, int>())
        .def("set_cameras",
             [](Reprojector& r, std::vector<py::dict> cams) {
                 std::vector<CameraParams> cs;
                 for (auto& d : cams) cs.push_back(to_cam(d));
                 r.set_cameras(cs);
             })
        .def("upload_images",
             [](Reprojector& r, py::array_t<float, py::array::c_style | py::array::forcecast> a) {
                 r.upload_images(a.data(), a.shape(0), a.shape(1), a.shape(2));
             })
        .def("render_bowl",
             [](Reprojector& r, py::dict vcam, float R0, float k, float Rmax) {
                 CameraParams v = to_cam(vcam);
                 BowlParams b{R0, k, Rmax};
                 auto out = py::array_t<float>({v.height, v.width, 4});
                 r.render_bowl(v, b, out.mutable_data());
                 return out;
             });
}
```
Create `cuda/src/libs/rendering_reprojector/bindings/CMakeLists.txt`:
```cmake
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(tpscuda tps_pybind.cpp)
target_link_libraries(tpscuda PRIVATE rendering_reprojector)
install(TARGETS tpscuda LIBRARY DESTINATION rendering_reprojector/python)
```
Add to `cuda/src/libs/CMakeLists.txt`: `if(BUILD_PYBIND)` `add_subdirectory(rendering_reprojector/bindings)` `endif()`. In `rendering_reprojector/CMakeLists.txt`, add `src/kernels/fill.cu` to the `add_library` sources.

- [ ] **Step 3: Write the round-trip parity test (RED)**

Create `tests/test_cuda_parity.py`:
```python
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
```

- [ ] **Step 4: Build + run the round-trip test**

Run: `cd cuda/scripts/libs_build && ./libs_build.sh Debug` (now builds `tpscuda`), then
`cd /home/ag7/Documents/TPSProjector && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_cuda_parity.py -v`
Expected: `test_toolchain_roundtrip` PASS (the constant fill round-trips CUDA→pybind→NumPy). If `tpscuda` import fails, confirm the install path and `pybind11_DIR` (set `-Dpybind11_DIR=$(python3 -c "import pybind11;print(pybind11.get_cmake_dir())")` in the build if `find_package(pybind11)` fails).

- [ ] **Step 5: Commit**

```bash
git add cuda/ tests/test_cuda_parity.py
git commit -m "feat(cuda): pybind11 module + CUDA->pybind->numpy round-trip"
```

---

### Task 3: Bowl reproject kernel + bowl parity vs NumpyRenderer

The first real correctness proof.

**Files:**
- Create: `cuda/src/libs/rendering_reprojector/src/kernels/{vecmath.cuh,surface.cuh,sampling.cuh,blend.cuh,reproject.cu}`
- Modify: `reprojector.cpp` (real `render_bowl` + device buffers), `reprojector` CMake (add `reproject.cu`)
- Modify: `tps_pybind.cpp` (no change needed — `render_bowl` already bound), `tests/test_cuda_parity.py` (add bowl parity)

**Interfaces:**
- Consumes the Task 1 API. Produces a working `render_bowl` numerically matching `NumpyRenderer`.

- [ ] **Step 1: Write device math helpers**

Create `cuda/src/libs/rendering_reprojector/src/kernels/vecmath.cuh`:
```cpp
#pragma once
#include <cuda_runtime.h>

namespace micropilot::rendering
{
__device__ __forceinline__ float3 vsub(float3 a, float3 b) { return make_float3(a.x-b.x,a.y-b.y,a.z-b.z); }
__device__ __forceinline__ float3 vadd(float3 a, float3 b) { return make_float3(a.x+b.x,a.y+b.y,a.z+b.z); }
__device__ __forceinline__ float3 vscale(float3 a, float s) { return make_float3(a.x*s,a.y*s,a.z*s); }
__device__ __forceinline__ float vdot(float3 a, float3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
__device__ __forceinline__ float3 vnorm(float3 a)
{
    float n = sqrtf(vdot(a, a)) + 1e-12f;
    return vscale(a, 1.0f / n);
}
}  // namespace micropilot::rendering
```
Create `surface.cuh`:
```cpp
#pragma once
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
__device__ __forceinline__ float bowl_height(float r, float R0, float k, float Rmax)
{
    float d = fminf(fmaxf(r - R0, 0.0f), Rmax - R0);
    return k * d * d;
}
__device__ __forceinline__ float bowl_g(float3 o, float3 dir, float t, float R0, float k, float Rmax)
{
    float3 P = vadd(o, vscale(dir, t));
    return P.z - bowl_height(sqrtf(P.x * P.x + P.y * P.y), R0, k, Rmax);
}
__device__ __forceinline__ bool intersect(float3 o, float3 dir, int surf_type, float flat_z0,
                                           float R0, float k, float Rmax, float3& P)
{
    if (surf_type == 0)
    {
        float dz = dir.z;
        if (fabsf(dz) <= 1e-12f) return false;
        float t = (flat_z0 - o.z) / dz;
        if (t <= 1e-9f) return false;
        P = vadd(o, vscale(dir, t));
        return true;
    }
    float eps = 1e-6f, tmax = 1.0e4f;
    if (!(bowl_g(o, dir, eps, R0, k, Rmax) > 0.0f && bowl_g(o, dir, tmax, R0, k, Rmax) < 0.0f))
        return false;
    float lo = eps, hi = tmax;
    for (int i = 0; i < 60; i++)
    {
        float mid = 0.5f * (lo + hi);
        if (bowl_g(o, dir, mid, R0, k, Rmax) > 0.0f) lo = mid;
        else hi = mid;
    }
    P = vadd(o, vscale(dir, 0.5f * (lo + hi)));
    return true;
}
}  // namespace micropilot::rendering
```
Create `sampling.cuh`:
```cpp
#pragma once
namespace micropilot::rendering
{
/** Bilinear sample of a (H,W,3) image with edge clamp (matches NumpyRenderer). */
__device__ __forceinline__ void bilinear(const float* img, int W, int H, float x, float y,
                                          float& r, float& g, float& b)
{
    x = fminf(fmaxf(x, 0.0f), W - 1.0f);
    y = fminf(fmaxf(y, 0.0f), H - 1.0f);
    int x0 = (int)floorf(x), y0 = (int)floorf(y);
    int x1 = min(x0 + 1, W - 1), y1 = min(y0 + 1, H - 1);
    float wx = x - x0, wy = y - y0;
    auto P = [&](int yy, int xx, int c) { return img[((yy)*W + (xx)) * 3 + c]; };
    for (int c = 0; c < 3; ++c)
    {
        float top = P(y0, x0, c) * (1 - wx) + P(y0, x1, c) * wx;
        float bot = P(y1, x0, c) * (1 - wx) + P(y1, x1, c) * wx;
        float v = top * (1 - wy) + bot * wy;
        if (c == 0) r = v; else if (c == 1) g = v; else b = v;
    }
}
}  // namespace micropilot::rendering
```
Create `blend.cuh`:
```cpp
#pragma once
namespace micropilot::rendering
{
__device__ __forceinline__ float smoothstep01(float x)
{
    x = fminf(fmaxf(x, 0.0f), 1.0f);
    return x * x * (3.0f - 2.0f * x);
}
__device__ __forceinline__ float border_feather(float u, float v, int W, int H, float margin)
{
    float d = fminf(fminf(u, (W - 1) - u), fminf(v, (H - 1) - v));
    if (margin <= 0.0f) return d >= 0.0f ? 1.0f : 0.0f;
    return smoothstep01(d / margin);
}
}  // namespace micropilot::rendering
```

- [ ] **Step 2: Write the bowl kernel**

Create `cuda/src/libs/rendering_reprojector/src/kernels/reproject.cu`:
```cpp
/** @file reproject.cu @brief Backward bowl reprojection kernel (port of NumpyRenderer). */
#include <cuda_runtime.h>

#include "kernels/blend.cuh"
#include "kernels/sampling.cuh"
#include "kernels/surface.cuh"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
/** Per-camera device record: K split + R columns + center. */
struct CamDev
{
    float fx, fy, cx, cy;
    float3 right, down, fwd, t;
    int w, h;
};

__global__ void bowl_kernel(float* out, int OW, int OH, const float* images, const CamDev* cams,
                            int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k,
                            float Rmax, float feather_margin, float fr, float fg, float fb)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= OW || y >= OH) return;
    int idx = (y * OW + x) * 4;

    float xc = (x - v.cx) / v.fx;
    float yc = (y - v.cy) / v.fy;
    float3 dc = make_float3(xc, yc, 1.0f);
    float3 dir = vnorm(make_float3(v.right.x * dc.x + v.down.x * dc.y + v.fwd.x * dc.z,
                                   v.right.y * dc.x + v.down.y * dc.y + v.fwd.y * dc.z,
                                   v.right.z * dc.x + v.down.z * dc.y + v.fwd.z * dc.z));
    float3 o = v.t;
    float3 P;
    if (!intersect(o, dir, surf_type, flat_z0, R0, k, Rmax, P))
    {
        out[idx] = fr; out[idx+1] = fg; out[idx+2] = fb; out[idx+3] = 0.0f;
        return;
    }
    float ar = 0, ag = 0, ab = 0, wsum = 0;
    for (int i = 0; i < ncam; ++i)
    {
        CamDev c = cams[i];
        float3 rel = vsub(P, c.t);
        float z = vdot(c.fwd, rel);
        if (z <= 1e-9f) continue;
        float xp = c.fx * vdot(c.right, rel) / z + c.cx;
        float yp = c.fy * vdot(c.down, rel) / z + c.cy;
        if (xp < 0 || xp > c.w - 1 || yp < 0 || yp > c.h - 1) continue;
        float r, g, b;
        bilinear(images + (size_t)i * c.w * c.h * 3, c.w, c.h, xp, yp, r, g, b);
        float align = fmaxf(0.0f, fminf(1.0f, vdot(vnorm(rel), c.fwd)));
        float w = border_feather(xp, yp, c.w, c.h, feather_margin) * align * align;
        ar += w * r; ag += w * g; ab += w * b; wsum += w;
    }
    if (wsum > 0.0f)
    {
        out[idx] = ar / wsum; out[idx+1] = ag / wsum; out[idx+2] = ab / wsum; out[idx+3] = 1.0f;
    }
    else { out[idx] = fr; out[idx+1] = fg; out[idx+2] = fb; out[idx+3] = 0.0f; }
}

void launch_bowl(float* d_out, int OW, int OH, const float* d_images, const CamDev* d_cams,
                 int ncam, CamDev v, int surf_type, float flat_z0, float R0, float k, float Rmax,
                 float feather_margin, float fr, float fg, float fb)
{
    dim3 block(16, 16);
    dim3 grid((OW + 15) / 16, (OH + 15) / 16);
    bowl_kernel<<<grid, block>>>(d_out, OW, OH, d_images, d_cams, ncam, v, surf_type, flat_z0, R0,
                                 k, Rmax, feather_margin, fr, fg, fb);
}
}  // namespace micropilot::rendering
```

- [ ] **Step 3: Implement real `render_bowl` in `reprojector.cpp`**

Replace `reprojector.cpp` with the real implementation: keep device buffers for images (`float* d_images`) and per-camera `CamDev` records (`CamDev* d_cams`), allocate on `set_cameras`/`upload_images`, and in `render_bowl` build the virtual `CamDev`, allocate `d_out` (OW*OH*4), `launch_bowl`, `cudaDeviceSynchronize`, copy back. Convert `CameraParams`→`CamDev`:
```cpp
// in an anonymous namespace in reprojector.cpp
static micropilot::rendering::/*forward-declared*/ /* use the struct from reproject.cu via a shared header */
```
Create a shared header `cuda/src/libs/rendering_reprojector/src/kernels/camdev.hpp` with the `CamDev` struct and `launch_bowl`/`launch_fill` declarations, include it from both `reproject.cu` and `reprojector.cpp` (move `struct CamDev` out of `reproject.cu` into this header; `reproject.cu` includes it). The converter:
```cpp
CamDev to_camdev(const CameraParams& c)
{
    CamDev d;
    d.fx = c.K[0]; d.fy = c.K[4]; d.cx = c.K[2]; d.cy = c.K[5];
    d.right = make_float3(c.R[0], c.R[3], c.R[6]);
    d.down  = make_float3(c.R[1], c.R[4], c.R[7]);
    d.fwd   = make_float3(c.R[2], c.R[5], c.R[8]);
    d.t = make_float3(c.t[0], c.t[1], c.t[2]);
    d.w = c.width; d.h = c.height;
    return d;
}
```
`render_bowl` uses `surf_type=1` (bowl) with `bowl.R0/k/Rmax`, `feather_margin=30.0f`, fill `(0,0,0)`. (`reprojector.cpp` must be compiled as CUDA or include `<cuda_runtime.h>` for `make_float3`/`cudaMalloc`; add `<cuda_runtime.h>`.) Store device image buffer sized `n*h*w*3` and per-camera `CamDev` array uploaded in `set_cameras`+`upload_images`.

- [ ] **Step 4: Add bowl parity test**

Append to `tests/test_cuda_parity.py`:
```python
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
```

- [ ] **Step 5: Build + run parity**

Run: `cd cuda/scripts/libs_build && ./libs_build.sh Debug` then
`cd /home/ag7/Documents/TPSProjector && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_cuda_parity.py -v`
Expected: `test_cuda_bowl_matches_numpy` PASS (PSNR > 40 dB vs NumpyRenderer). If low, check the R-column mapping and the ray basis (`v.right*xc + v.down*yc + v.fwd`), and that the output is NOT flipped (CUDA writes row 0 = top directly).

- [ ] **Step 6: Commit**

```bash
git add cuda/ tests/test_cuda_parity.py
git commit -m "feat(cuda): bowl reproject kernel + parity vs NumpyRenderer (>40dB)"
```

---

### Task 4: Depth splat kernel + parity vs DepthRenderer

**Files:**
- Create: `cuda/src/libs/rendering_reprojector/src/kernels/splat.cu`
- Modify: `camdev.hpp` (declare `launch_splat`), `reprojector.cpp` (real `render_depth` + `upload_depth`), `tps_pybind.cpp` (bind `upload_depth` + `render_depth`), `tests/test_cuda_parity.py`

**Interfaces:**
- Produces a working `render_depth` matching `DepthRenderer(splat_radius=1)`.

- [ ] **Step 1: Write the splat kernel (nearest-wins via packed atomicMin)**

Create `cuda/src/libs/rendering_reprojector/src/kernels/splat.cu`:
```cpp
/** @file splat.cu @brief Forward point-cloud splat with packed-atomicMin z-buffer. */
#include <cuda_runtime.h>
#include <stdint.h>

#include "kernels/camdev.hpp"
#include "kernels/vecmath.cuh"

namespace micropilot::rendering
{
// Pack (float depth bits | point index). For z>0, IEEE float bits are monotonic,
// so atomicMin on the u64 selects the nearest point; low 32 bits index the winner.
__global__ void zbuf_init(unsigned long long* zbuf, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) zbuf[i] = 0xFFFFFFFFFFFFFFFFULL;
}

__global__ void splat_kernel(unsigned long long* zbuf, int OW, int OH, const float* pts,
                             int npts, CamDev v, int radius)
{
    int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p >= npts) return;
    float3 P = make_float3(pts[p * 3], pts[p * 3 + 1], pts[p * 3 + 2]);
    float3 rel = vsub(P, v.t);
    float z = vdot(v.fwd, rel);
    if (z <= 1e-6f) return;
    float xp = v.fx * vdot(v.right, rel) / z + v.cx;
    float yp = v.fy * vdot(v.down, rel) / z + v.cy;
    int cx = (int)lrintf(xp), cy = (int)lrintf(yp);
    unsigned int zbits = __float_as_uint(z);
    unsigned long long packed = ((unsigned long long)zbits << 32) | (unsigned int)p;
    for (int dy = -radius; dy <= radius; ++dy)
        for (int dx = -radius; dx <= radius; ++dx)
        {
            int x = cx + dx, y = cy + dy;
            if (x < 0 || x >= OW || y < 0 || y >= OH) continue;
            atomicMin(&zbuf[y * OW + x], packed);
        }
}

__global__ void resolve_kernel(const unsigned long long* zbuf, const float* cols, float* out,
                               int n, float fr, float fg, float fb)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    unsigned long long z = zbuf[i];
    if (z == 0xFFFFFFFFFFFFFFFFULL)
    {
        out[i*4] = fr; out[i*4+1] = fg; out[i*4+2] = fb; out[i*4+3] = 0.0f;
        return;
    }
    unsigned int p = (unsigned int)(z & 0xFFFFFFFFULL);
    out[i*4] = cols[p*3]; out[i*4+1] = cols[p*3+1]; out[i*4+2] = cols[p*3+2]; out[i*4+3] = 1.0f;
}

void launch_splat(unsigned long long* d_zbuf, float* d_out, int OW, int OH, const float* d_pts,
                  const float* d_cols, int npts, CamDev v, int radius, float fr, float fg, float fb)
{
    int n = OW * OH, t = 256;
    zbuf_init<<<(n + t - 1) / t, t>>>(d_zbuf, n);
    splat_kernel<<<(npts + t - 1) / t, t>>>(d_zbuf, OW, OH, d_pts, npts, v, radius);
    resolve_kernel<<<(n + t - 1) / t, t>>>(d_zbuf, d_cols, d_out, n, fr, fg, fb);
}
}  // namespace micropilot::rendering
```
(Move `CamDev` + all `launch_*` declarations into `camdev.hpp`; include it here.)

- [ ] **Step 2: Host point cloud in `reprojector.cpp`**

`upload_depth(nhw, n, h, w)` + the stored images build the world point cloud on the host exactly like `DepthRenderer._point_cloud`: for each camera and each finite-depth pixel, back-project `(u,v,depth)` to world via that camera's `CameraParams` (the inverse of `project`: `x=(u-cx)/fx*z; y=(v-cy)/fy*z; world = R@[x,y,z]+t`), collect points (xyz) and colors (rgb from the image). Upload `d_pts` (M,3) and `d_cols` (M,3). `render_depth` allocates a `unsigned long long` zbuf (OW*OH), calls `launch_splat` with `radius=splat_radius`, copies the resolved `d_out` back. Use `depth==inf` (non-finite) to skip, matching Python.

- [ ] **Step 3: Bind + test**

In `tps_pybind.cpp` add `.def("upload_depth", ...)` (NHW float array) and `.def("render_depth", [](Reprojector& r, py::dict v, int radius){...})` returning `(H,W,4)`. Append to `tests/test_cuda_parity.py`:
```python
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
```

- [ ] **Step 4: Build + run; Step 5: Commit**

Run the build + `tests/test_cuda_parity.py`; expect depth parity PASS. Commit:
```bash
git add cuda/ tests/test_cuda_parity.py
git commit -m "feat(cuda): depth splat kernel (packed-atomicMin z-buffer) + parity vs DepthRenderer"
```

---

### Task 5: Hybrid composite + parity

**Files:** Modify `reprojector.cpp` (`render_hybrid`), `tps_pybind.cpp` (bind), `tests/test_cuda_parity.py`.

- [ ] **Step 1:** Implement `render_hybrid`: render depth into a device RGBA buffer and bowl into another, then a small `hybrid_kernel` (add to `splat.cu` or a new `composite.cu`) that, per pixel, takes depth RGBA where depth A>0.5 else bowl RGBA, A = depth.A || bowl.A. Bind `render_hybrid(v, R0,k,Rmax, radius)`.
- [ ] **Step 2:** Append parity test mirroring the Python hybrid: `env = where(depth_valid, depth, bowl)` (the `Engine._render_env` rule). Assert `>28 dB` vs that composite over the same setup.
- [ ] **Step 3:** Build + run + commit `feat(cuda): hybrid composite + parity`.

(Full code analogous to Tasks 3–4; the composite kernel is: `out = dA>0.5 ? depth : bowl; outA = max(dA,bA)`.)

---

### Task 6: C++ GTest golden + standalone example + find_package consumability

**Files:** Create `cuda/tools/export_golden.py` (dumps a reference frame), `cuda/src/libs/rendering_reprojector/tests/test_golden.cpp`, `cuda/examples/{render_demo.cpp,CMakeLists.txt}`; modify test CMake.

- [ ] **Step 1:** `export_golden.py` runs `NumpyRenderer` on the fixed setup and writes `golden_bowl.bin` (raw float32 `(H,W,3)`) + `golden_bowl.txt` (H W on one line) + the camera/rig params as a simple text file the C++ test reads.
- [ ] **Step 2:** `test_golden.cpp` (GTest) loads the params + golden, builds a `Reprojector`, renders bowl, computes PSNR vs golden in-process, `EXPECT_GT(psnr, 40.0)`. Add to the lib test target.
- [ ] **Step 3:** `render_demo.cpp` is a **headless pure-C++ consumer** (consumer mode 2): a `main()` that `find_package`-style links `micropilot_rendering::reprojector`, renders a short sequence of virtual poses, and writes each frame to a numbered PPM (`frame_000.ppm`, …) — proving standalone (no-Python, no-GL, no-display) consumability and the render-to-file path. A header comment notes that swapping the PPM write for an OpenCV `cv::VideoWriter` yields a video (the production pure-C++ app), kept out of the example to avoid an OpenCV dependency. A separate tiny CMake project under `cuda/examples/` that does `find_package(micropilot_rendering REQUIRED)` against the install tree and links nothing GL/windowing.
- [ ] **Step 4:** Build (`./libs_build.sh Debug`), run `ctest` (golden passes), configure+build the example against `cuda/install`, run it. Commit `feat(cuda): C++ golden test + standalone find_package example`.

---

### Task 7: CUDA benchmark + packaging verification

**Files:** Create `cuda/tools/benchmark.cpp` (or a pybind-driven `tests/test_cuda_benchmark.py` marked slow).

- [ ] **Step 1:** Benchmark `render_bowl` at 1280×720 over N iters with `cudaDeviceSynchronize` timing; report fps. Compare against the GL 882 fps number (informational; assert a generous floor e.g. `>200` to catch regressions).
- [ ] **Step 2:** Verify the install tree is consumable: `find_package(micropilot_rendering)` from a clean throwaway dir resolves `micropilot_rendering::reprojector`. Commit `perf(cuda): benchmark + packaging verification`.

---

### Task 8: ROS2 LifecycleNode + launch smoke test + README

Manual-verify (node loop). Builds with colcon; the launch smoke test is the integration gate.

**Files:** Create `cuda/src/ros_apps/src/micropilot_rendering_node/{package.xml,CMakeLists.txt,include/micropilot_rendering_node/rendering_node.hpp,src/rendering_node.cpp,src/main.cpp,launch/rendering_node.launch.py,test/smoke_test.py}`; create `cuda/scripts/ros_apps_build/{colcon_build.sh,config_colcon.yaml}`; update `README.md`.

- [ ] **Step 1:** Write `package.xml` (format 3, ament_cmake, depends rclcpp, rclcpp_lifecycle, sensor_msgs, cv_bridge, tf2_ros, the rendering lib via `<depend>`), and the node CMake that `find_package(micropilot_rendering REQUIRED)` and links `micropilot_rendering::reprojector` + ament deps.
- [ ] **Step 2:** `rendering_node.hpp/.cpp`: `class RenderingNode : public rclcpp_lifecycle::LifecycleNode` in `micropilot::rendering_app`. `on_configure` reads params (camera topics list, virtual-pose preset, bowl params), builds `Reprojector`, sets cameras from CameraInfo + tf2 extrinsics; subscribes N `Image`+`CameraInfo`; on a sync/timer calls `render_bowl`, converts via cv_bridge, publishes `Image`+`CameraInfo`. Report component_health.
- [ ] **Step 3:** `colcon_build.sh` + `config_colcon.yaml` mirror manager's. Build: `cd cuda/scripts/ros_apps_build && ./colcon_build.sh micropilot_rendering_node`. (Source ROS 2 Humble first: `source /opt/ros/humble/setup.bash`.)
- [ ] **Step 4:** `smoke_test.py` launch test: publish synthetic `Image`+`CameraInfo` on N topics with known calibration + static tf, configure+activate the node, assert it publishes a synthesized `Image` whose pixels are not all-zero. This is the integration gate. (Do NOT block on a manual GUI; the test is headless.)
- [ ] **Step 5:** Update `README.md` roadmap: C++/CUDA library implemented (bowl/depth/hybrid), micropilot-convention package, pybind parity numbers, ROS2 node. Commit `feat(cuda): ROS2 LifecycleNode + launch smoke test + README`.

---

## Self-Review

**Spec coverage:** §5 layout → Tasks 1,2,8. §6 API+kernels → Tasks 1,3,4,5. §7 CMake package → Task 1 (+example consume Task 6). §8 pybind parity → Tasks 2–5. §9 ROS node → Task 8. §10 testing (pybind, GTest golden, ROS smoke, benchmark) → Tasks 3–8. §11 build scripts → Tasks 1,8. §12 build order → task sequence. All covered.

**Placeholder scan:** Tasks 1–4 carry complete code. Tasks 5–8 give complete code for novel parts (composite kernel rule, golden format, node class shape) and reference Tasks 3–4's verbatim patterns for the mechanical remainder — flagged explicitly, not "TODO". The ROS node (Task 8) is the largest prose-spec'd task; its lib usage is fully specified, ROS plumbing follows the micropilot LifecycleNode pattern the Explore report documented.

**Type consistency:** `CameraParams{K[9],R[9],t[3],width,height}`, `BowlParams{R0,k,Rmax}`, `CamDev` (in `camdev.hpp`), and the `launch_bowl`/`launch_splat` signatures are consistent across `reproject.cu`, `splat.cu`, `reprojector.cpp`. The pybind `render_bowl(vcam, R0,k,Rmax)` / `render_depth(vcam, radius)` signatures match the test calls. Namespace `micropilot::rendering` throughout; package/target names `micropilot_rendering` / `micropilot_rendering::reprojector` consistent.

**Known intentional choices:** manual bilinear (exact NumPy parity); CUDA writes row-0-top directly (no flip); float32 vs float64 → 40/28 dB tolerances; ROS node manual-verify with a headless launch smoke gate; Tasks 5/8 reference earlier verbatim patterns rather than re-pasting identical kernel boilerplate.
