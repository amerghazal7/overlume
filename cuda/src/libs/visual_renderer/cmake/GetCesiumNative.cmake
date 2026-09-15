# GetCesiumNative.cmake — fetches and builds the pinned cesium-native release
# from source, under THIS project's clang/libc++ toolchain.
#
# Pin (Epic 6, VM-061 — see docs/superpowers/plans/2026-08-18-visual-mode-epic6.md):
#   CESIUM_NATIVE_VERSION = 0.64.0 (git tag v0.64.0, 2026-09-01). cesium-native
#   publishes git tags only — no prebuilt binary SDK exists — so unlike
#   GetFilament.cmake this is a SOURCE pin (tarball URL + sha256), and the
#   prebuilt-glibc hazard that forced Filament's bisect does not apply: we
#   compile everything here with our own toolchain. Requirements at this tag
#   (upstream CHANGES.md / doc/topics/developer-setup.md): C++20 (since
#   v0.42.0), CMake 3.15+, Clang 12+. Dependencies come from vcpkg in
#   manifest mode (vcpkg.json), bootstrapped by EZVCPKG at configure time;
#   the overlay triplet below forces every port onto clang/libc++ so no
#   gcc/libstdc++ object ever enters this library's link (ADR-0003).
#   Re-verify the toolchain build + the node-side symbol audit
#   (scripts/merge_yamlcpp.sh header) before ever bumping — don't assume
#   newer-is-safer.
#
# SHA256 provenance (VM-061 Step 0): computed via `sha256sum` on a
# `curl -L` download of the tag tarball, cross-checked against a SECOND,
# independent download (via codeload.github.com's direct tar.gz path rather
# than the archive/refs/tags redirect) on 2026-09-15 — both agreed. Never
# fabricated; see the task report for the exact commands run.
set(CESIUM_NATIVE_VERSION "0.64.0")
set(CESIUM_NATIVE_URL
    "https://github.com/CesiumGS/cesium-native/archive/refs/tags/v${CESIUM_NATIVE_VERSION}.tar.gz")
set(CESIUM_NATIVE_SHA256
    "f3629345db4cb7412380cc31dea502aeb9e2eca75129ccbc04b7628970031c11")

# Force every vcpkg port onto clang/libc++ (Decision 2 in the epic plan).
set(VCPKG_OVERLAY_TRIPLETS "${CMAKE_CURRENT_LIST_DIR}/vcpkg-triplets" CACHE STRING "" FORCE)
set(VCPKG_TARGET_TRIPLET "x64-linux-clang-libcxx" CACHE STRING "" FORCE)
# NOT honored by EZVCPKG: the actual invocation (see build-cesium/logs/configure.log)
# passes -D_HOST_TRIPLET=x64-linux regardless of this setting -- EZVCPKG reads
# VCPKG_TARGET_TRIPLET and VCPKG_OVERLAY_TRIPLETS but not this variable. Kept
# (rather than dropped) as documentation of intent; harmless because the host
# triplet only ever installs vcpkg-cmake/vcpkg-cmake-config script helpers, so
# nothing gcc/libstdc++-built from that tree reaches this library's link.
set(VCPKG_HOST_TRIPLET "x64-linux-clang-libcxx" CACHE STRING "" FORCE)

# No tests, no tools — the YAML_CPP_BUILD_TOOLS lesson (CMakeLists.txt:89-93).
set(CESIUM_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(CESIUM_COVERAGE_ENABLED OFF CACHE BOOL "" FORCE)
# Same lesson: no third-party lint targets, no install pollution in our build —
# all three default ON at v0.64.0 (upstream top-level CMakeLists.txt:226-228,302
# — option names re-verified against the pinned tag at implementation).
set(CESIUM_ENABLE_CLANG_TIDY OFF CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_STATIC_LIBS OFF CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_HEADERS OFF CACHE BOOL "" FORCE)

# cesium-native's cmake/macros/configure_cesium_library.cmake sets
# COMPILE_WARNING_AS_ERROR YES plus -Wconversion/-Wsign-conversion/-Wshadow
# on its own targets; we are not upstream's CI and do not gate our build on
# their warning set under a compiler (clang-14/libc++-14) they never test.
#
# NOTE (deviation recorded, verified against the pinned tree at
# implementation): setting CMAKE_COMPILE_WARNING_AS_ERROR here is necessary
# but, on CMake >= 3.24 (this box: 3.28.3), NOT sufficient on its own --
# configure_cesium_library() calls set_target_properties(<tgt> PROPERTIES
# COMPILE_WARNING_AS_ERROR YES) explicitly on every cesium target, which
# overrides the global variable's initial-value role regardless of set
# order. The FORCE_OFF loop after FetchContent_MakeAvailable below is the
# actual mechanism that neutralizes it; this variable is kept too since it's
# the plan's documented intent and costs nothing.
set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)

# DEVIATION (recorded, root-caused empirically, VM-061 Step 1): vcpkg ships
# a built-in pseudo-port, scripts/detect_compiler, that it runs ONCE per
# `vcpkg install` invocation (before touching our target triplet at all) to
# fingerprint "the ambient default compiler" for its binary-caching cache
# key. Captured its exact invocation from
# buildtrees/detect_compiler/stdout-x64-linux.log: it is hardcoded to
# -DVCPKG_TARGET_TRIPLET=x64-linux (the stock COMMUNITY triplet, not our
# x64-linux-clang-libcxx overlay) chainloading vcpkg's own
# scripts/toolchains/linux.cmake (not our vcpkg-clang-libcxx-toolchain.cmake)
# -- entirely independent of the VCPKG_TARGET_TRIPLET/VCPKG_OVERLAY_TRIPLETS
# we set above, so nothing in our overlay triplet can reach it. That stock
# toolchain applies NO -stdlib flag, and CMake's own default CXX-compiler
# search for this probe resolves to this project's clang++ (confirmed via
# the probe's own captured CMakeCache.txt) -- clang's default (no -stdlib
# flag) then selects "GCC installation" gcc-12 for its C++ runtime, which
# has no libstdc++-dev package on this box (only gcc-11 does), so a plain
# `-lstdc++` fails to resolve at link time ("cannot find -lstdc++"),
# independent of any flag we could inject into this specific probe (it is
# vcpkg's own hardcoded invocation, not reachable via CMAKE_CXX_FLAGS_INIT,
# VCPKG_CHAINLOAD_TOOLCHAIN_FILE, or a project-level CMAKE_CXX_COMPILER
# override -- all three were tried and confirmed NOT to reach this probe).
#
# Fix: the actual missing piece is link-time only (this probe's test program
# is a bare `int main(){return 0;}`, no headers, so header availability is
# irrelevant) -- exporting LIBRARY_PATH (a plain env var both gcc's and
# clang's linker driver honor as extra `-L` search dirs, no compiler flag
# needed) to gcc-11's install dir, which DOES ship libstdc++.so, makes a
# bare `clang++ -fPIC trivial.cxx -o t` (no -stdlib flag at all) link
# successfully -- confirmed directly, exit 0. LIBRARY_PATH is a real
# environment variable, so it propagates through every process boundary
# (ezvcpkg's execute_process -> the vcpkg binary -> its own nested cmake/
# ninja invocations) without needing to intercept any one of them
# specifically. Computed from whichever installed gcc actually ships the
# libstdc++.so dev symlink (glob, not a hardcoded version number) so this
# keeps working if the system's gcc versions change.
file(GLOB _mpviz_cn_libstdcxx_dev_dirs "/usr/lib/gcc/*/*")
set(_mpviz_cn_libstdcxx_dev_dir "")
foreach(_d ${_mpviz_cn_libstdcxx_dev_dirs})
    if(EXISTS "${_d}/libstdc++.so" AND IS_DIRECTORY "${_d}")
        set(_mpviz_cn_libstdcxx_dev_dir "${_d}")
        break()
    endif()
endforeach()
if(_mpviz_cn_libstdcxx_dev_dir)
    set(ENV{LIBRARY_PATH} "${_mpviz_cn_libstdcxx_dev_dir}:$ENV{LIBRARY_PATH}")
else()
    message(WARNING
        "visual_renderer/GetCesiumNative: no /usr/lib/gcc/*/*/libstdc++.so "
        "dev symlink found anywhere -- vcpkg's own scripts/detect_compiler "
        "pseudo-port (see comment above) may fail to link its bare "
        "no-flags compiler probe on this box.")
endif()

# vcpkg ports/cesium-native's own build occasionally RUN a build-time tool
# they just linked (codegen, feature probes) dynamically against libc++ --
# our rootless toolchain prefix's libc++.so/.so.1, libc++abi.so, libunwind.so
# live under a directory that mimics /usr/lib/x86_64-linux-gnu/ but isn't
# actually on the system loader's search path, so a bare `execute_process`
# invocation of such a tool would fail with "cannot open shared object
# file" unless LD_LIBRARY_PATH points at it. _libcxx_lib_dir is computed by
# this project's own top-level CMakeLists.txt (above, before
# include(cmake/GetFilament.cmake)) from the SAME compiler this file's
# overlay-triplet toolchain resolves, so it's the correct directory here too.
set(ENV{LD_LIBRARY_PATH} "${_libcxx_lib_dir}:$ENV{LD_LIBRARY_PATH}")

set(CMAKE_FIND_PACKAGE_TARGETS_GLOBAL TRUE)
include(FetchContent)
FetchContent_Declare(
    cesium-native
    URL "${CESIUM_NATIVE_URL}"
    URL_HASH SHA256=${CESIUM_NATIVE_SHA256})
FetchContent_MakeAvailable(cesium-native)

# See the NOTE above: force off the per-target COMPILE_WARNING_AS_ERROR
# property that configure_cesium_library() sets explicitly on every cesium
# subdirectory target, recursively, across every directory FetchContent just
# added. get_property(... BUILDSYSTEM_TARGETS) is scoped per-directory, so
# walk SUBDIRECTORIES recursively from the fetched source root.
function(_mpviz_disable_warnings_as_errors_recursive _dir)
    get_property(_targets DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_tgt ${_targets})
        set_target_properties(${_tgt} PROPERTIES COMPILE_WARNING_AS_ERROR OFF)
    endforeach()
    get_property(_subdirs DIRECTORY "${_dir}" PROPERTY SUBDIRECTORIES)
    foreach(_subdir ${_subdirs})
        _mpviz_disable_warnings_as_errors_recursive("${_subdir}")
    endforeach()
endfunction()
_mpviz_disable_warnings_as_errors_recursive("${cesium-native_SOURCE_DIR}")
