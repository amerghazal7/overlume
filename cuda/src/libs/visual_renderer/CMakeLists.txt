cmake_minimum_required(VERSION 3.18)

# visual_renderer is its own CMake project (not add_subdirectory'd into the
# gcc-toolchain `cuda/` build): the toolchain boundary in the design spec
# (docs/superpowers/specs/2026-08-18-visual-mode-design.md §2) requires this
# library to compile clang/libc++ while the rest of the CUDA/ROS stack stays
# on gcc/libstdc++ — CMAKE_CXX_COMPILER is fixed per top-level project, so the
# split has to happen at this granularity. Consumers (the ROS node) link
# against it only through the POD-only public header in include/.
project(visual_renderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Configure with `cmake --toolchain cmake/toolchain-clang-libcxx.cmake` (see
# that file + scripts/setup_toolchain.sh + README.md) — it selects a working
# clang++/libc++ and bakes in -stdlib=libc++. These are defensive checks in
# case someone configures by hand instead.
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR
        "visual_renderer must be built with clang++ (got CMAKE_CXX_COMPILER_ID="
        "'${CMAKE_CXX_COMPILER_ID}'). Configure with:\n"
        "  cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S . \n"
        "(run scripts/setup_toolchain.sh first if you have no system clang++/libc++). "
        "See README.md.")
endif()
if(NOT CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    message(FATAL_ERROR
        "visual_renderer must be built against libc++ (pass "
        "-DCMAKE_CXX_FLAGS=-stdlib=libc++), which the toolchain file above "
        "already does. See spec §2 for why gcc/libstdc++ and clang/libc++ "
        "must not mix.")
endif()

# visual_renderer links libc++/libc++abi/libunwind as explicit static
# archives (below) rather than the driver's default dynamic -lc++, so the
# result runs with no LD_LIBRARY_PATH/rpath needed (this used to bite us:
# `filament_link_probe` failed at runtime with "libc++.so.1: cannot open
# shared object file" under a plain ctest with nothing hand-exported).
#
# Resolve the runtime lib dir from the compiler itself (never hardcode it —
# it may be a root-less local prefix, not a fixed install path) via
# -print-file-name, the same mechanism CMake itself uses. libc++abi.a and
# libunwind.a are then required as siblings of libc++.a in that same
# directory rather than resolved independently via -print-file-name/-l:
# this box also has an unrelated, ABI-incompatible system package that is
# ALSO named libunwind (the nongnu libunwind used for backtraces, not
# LLVM's compiler-rt libunwind that libc++abi needs) and clang's default
# search path resolves a bare `-lunwind`/-print-file-name=libunwind.a to
# THAT one (confirmed) — sibling-of-libc++.a is the only reliable way to
# get the matching one.
execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -stdlib=libc++ -print-file-name=libc++.a
    OUTPUT_VARIABLE _libcxx_a
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_libcxx_a "${_libcxx_a}" REALPATH)
if(NOT EXISTS "${_libcxx_a}")
    message(FATAL_ERROR
        "visual_renderer: could not resolve a static libc++.a from compiler "
        "'${CMAKE_CXX_COMPILER}' (looked at '${_libcxx_a}'). Is "
        "CMAKE_CXX_COMPILER a real clang++ with a co-located libc++-dev?")
endif()
get_filename_component(_libcxx_lib_dir "${_libcxx_a}" DIRECTORY)
set(_libcxxabi_a "${_libcxx_lib_dir}/libc++abi.a")
set(_libunwind_a "${_libcxx_lib_dir}/libunwind.a")
foreach(_rt_lib "${_libcxxabi_a}" "${_libunwind_a}")
    if(NOT EXISTS "${_rt_lib}")
        message(FATAL_ERROR
            "visual_renderer: expected static runtime lib not found: "
            "${_rt_lib} (sibling of ${_libcxx_a}). Toolchain layout mismatch?")
    endif()
endforeach()

include(cmake/GetFilament.cmake)

# GLOB with CONFIGURE_DEPENDS so later tasks (Epic 0 Task 2+) that add files
# under src/, tests/, examples/ don't need to touch this CMakeLists.txt too —
# a fresh `cmake` re-run picks them up.
file(GLOB_RECURSE VISUAL_RENDERER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(visual_renderer STATIC ${VISUAL_RENDERER_SOURCES})
add_library(micropilot_visualization::visual_renderer ALIAS visual_renderer)

target_include_directories(visual_renderer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# PRIVATE: Filament::filament's headers (and the raw static archives are
# still forwarded to any final executable regardless — that's how CMake
# handles static-lib link dependencies) must never leak to the gcc/
# libstdc++ ROS node that links this library — it only ever sees the POD
# api.h include/ dir above. See spec §2 / api.h's header comment.
target_link_libraries(visual_renderer PRIVATE
    Filament::filament
    "${_libcxx_a}"
    "${_libcxxabi_a}"
    "${_libunwind_a}"
)

enable_testing()

add_test(NAME check_pod_header
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_pod_header.sh)

# Build-integration proof (Epic 0 Task 1): actually links `visual_renderer`
# (and therefore the ~30 cross-referencing Filament static archives pulled in
# by cmake/GetFilament.cmake) into a real executable and exercises a call
# that spans that archive graph. check_pod_header.sh proves the header is
# POD-clean; this proves the link recipe itself is sound — neither the
# static-lib-only build above nor the header check ever invokes the linker
# against Filament's archives. See smoke/filament_link_probe.cpp.
add_executable(filament_link_probe smoke/filament_link_probe.cpp)
# This probe deliberately calls Filament directly (see file header comment)
# to prove the archive-link-order fix, so it needs Filament's headers itself
# — granted explicitly here, NOT inherited from visual_renderer (whose
# Filament dependency is PRIVATE precisely so ordinary consumers, i.e. the
# eventual gcc ROS node, never see them).
target_include_directories(filament_link_probe PRIVATE "${FILAMENT_ROOT}/include")
target_link_libraries(filament_link_probe PRIVATE visual_renderer)
add_test(NAME filament_link_probe COMMAND filament_link_probe)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB _visual_renderer_test_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
        foreach(_test_src ${_visual_renderer_test_srcs})
            get_filename_component(_test_name "${_test_src}" NAME_WE)
            add_executable(${_test_name} "${_test_src}")
            target_link_libraries(${_test_name} PRIVATE visual_renderer GTest::GTest GTest::Main)
            include(GoogleTest)
            gtest_discover_tests(${_test_name} DISCOVERY_TIMEOUT 30)
        endforeach()
    else()
        message(STATUS "visual_renderer: GTest not found; skipping lib tests.")
    endif()
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    file(GLOB _visual_renderer_example_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach(_example_src ${_visual_renderer_example_srcs})
        get_filename_component(_example_name "${_example_src}" NAME_WE)
        add_executable(${_example_name} "${_example_src}")
        target_link_libraries(${_example_name} PRIVATE visual_renderer)
    endforeach()
endif()
