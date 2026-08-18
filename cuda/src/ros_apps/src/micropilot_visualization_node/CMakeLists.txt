cmake_minimum_required(VERSION 3.22)
project(micropilot_visualization_node LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# SetVirtualCam.srv (plan Task 5 / spec §6) — existing srv package, a
# dependency not a copy; micropilot_rendering_node is a
# rosidl_interface_packages member so find_package resolves its generated
# C++ headers (micropilot_rendering_node/srv/set_virtual_cam.hpp).
find_package(micropilot_rendering_node REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  micropilot_rendering_node)

# ── visual_renderer (clang/libc++ static lib) — cross-toolchain import ──────
# visual_renderer (docs/superpowers/plans/2026-08-18-visual-mode.md Epic 0
# Task 1/2) builds standalone with clang/libc++ (see its README.md) and
# cannot be add_subdirectory'd here: this package builds stock gcc/libstdc++
# (Global Constraints / spec §2 — mixing the two runtimes at a std:: boundary
# is UB). It also has no install()/find_package config of its own (this node
# is its first consumer), so this package imports the already-built static
# archive directly instead of via find_package.
#
# Why this actually links: an Itanium-ABI static archive links into a gcc
# binary regardless of which compiler produced it (same ABI, same target
# triple) as long as nothing std:: crosses the boundary — which is exactly
# what api.h's POD-only contract guarantees. libc++'s `std::__1` inline
# namespace keeps its mangled symbols distinct from this package's
# libstdc++ ones, so no duplicate-symbol clash at final link time.
set(VISUAL_RENDERER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/visual_renderer")
get_filename_component(VISUAL_RENDERER_DIR "${VISUAL_RENDERER_DIR}" ABSOLUTE)
set(VISUAL_RENDERER_LIB "${VISUAL_RENDERER_DIR}/build/libvisual_renderer.a")
if(NOT EXISTS "${VISUAL_RENDERER_LIB}")
    message(FATAL_ERROR
        "visual_renderer is not built. Build it first (see "
        "${VISUAL_RENDERER_DIR}/README.md):\n"
        "  cd ${VISUAL_RENDERER_DIR}\n"
        "  scripts/setup_toolchain.sh\n"
        "  cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .\n"
        "  cmake --build build\n"
        "then re-run this configure.")
endif()

# Reuse visual_renderer's own cmake/GetFilament.cmake (unmodified — just
# include()'d) instead of re-declaring its ~30-archive link recipe here.
# Pre-seed FILAMENT_ROOT to the SDK that lib's own build already fetched and
# sha256-verified, so this second configure doesn't re-download ~50 MB: the
# script's own `set(FILAMENT_ROOT ... CACHE PATH ...)` is a no-op once the
# variable is already in the cache, and it skips the fetch when
# FILAMENT_ROOT/include/filament/Engine.h already exists.
set(FILAMENT_VERSION "1.56.5")
set(FILAMENT_ROOT "${VISUAL_RENDERER_DIR}/build/_deps/filament-${FILAMENT_VERSION}/filament"
    CACHE PATH "Filament SDK root (reused from visual_renderer's own build)")
include("${VISUAL_RENDERER_DIR}/cmake/GetFilament.cmake")

# Locate the same rootless clang toolchain visual_renderer's own
# scripts/setup_toolchain.sh bootstraps, purely as a TOOL to resolve the
# static libc++/libc++abi/libunwind archive paths below — this package's own
# sources still compile with gcc throughout; nothing here recompiles with
# clang. See visual_renderer/CMakeLists.txt for why sibling-of-libc++.a (not
# a bare -lunwind) is required: this box's system libunwind package is an
# unrelated, ABI-incompatible library of the same name.
find_program(MPVIZ_CLANGXX clang++
    PATHS "$ENV{XDG_CACHE_HOME}/mpviz-toolchain/bin" "$ENV{HOME}/.cache/mpviz-toolchain/bin"
    NO_DEFAULT_PATH)
if(NOT MPVIZ_CLANGXX)
    find_program(MPVIZ_CLANGXX clang++)
endif()
if(NOT MPVIZ_CLANGXX)
    message(FATAL_ERROR
        "No clang++ found (checked visual_renderer's rootless toolchain "
        "prefix and PATH). Run ${VISUAL_RENDERER_DIR}/scripts/setup_toolchain.sh "
        "first (it is idempotent).")
endif()

execute_process(
    COMMAND "${MPVIZ_CLANGXX}" -stdlib=libc++ -print-file-name=libc++.a
    OUTPUT_VARIABLE _libcxx_a
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_libcxx_a "${_libcxx_a}" REALPATH)
if(NOT EXISTS "${_libcxx_a}")
    message(FATAL_ERROR
        "Could not resolve a static libc++.a from '${MPVIZ_CLANGXX}' "
        "(looked at '${_libcxx_a}').")
endif()
get_filename_component(_libcxx_lib_dir "${_libcxx_a}" DIRECTORY)
set(_libcxxabi_a "${_libcxx_lib_dir}/libc++abi.a")
set(_libunwind_a "${_libcxx_lib_dir}/libunwind.a")
foreach(_rt_lib "${_libcxxabi_a}" "${_libunwind_a}")
    if(NOT EXISTS "${_rt_lib}")
        message(FATAL_ERROR
            "Expected static runtime lib not found: ${_rt_lib} (sibling of ${_libcxx_a}).")
    endif()
endforeach()

add_library(visual_renderer_prebuilt STATIC IMPORTED)
set_target_properties(visual_renderer_prebuilt PROPERTIES
    IMPORTED_LOCATION "${VISUAL_RENDERER_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${VISUAL_RENDERER_DIR}/include"
    INTERFACE_LINK_LIBRARIES "Filament::filament;${_libcxx_a};${_libcxxabi_a};${_libunwind_a}")

# ── Node library ─────────────────────────────────────────────────────────────
add_library(visualization_node_lib SHARED
  src/visualization_node.cpp)

target_include_directories(visualization_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(visualization_node_lib ${THIS_PACKAGE_DEPS})
target_link_libraries(visualization_node_lib visual_renderer_prebuilt)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(visualization_node src/main.cpp)
target_link_libraries(visualization_node visualization_node_lib)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS visualization_node_lib visualization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
