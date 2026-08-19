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

# yaml-cpp (Epic 1 Task 2 / VM-011): theme.cpp parses assets/themes/*.yaml.
# FetchContent'd and built with this project's own clang/libc++ toolchain for
# the same ABI-safety reason as GoogleTest below (a prebuilt system
# libyaml-cpp would be gcc/libstdc++). Linked PRIVATE into visual_renderer —
# never leaks past the POD api.h/scene.h boundary.
include(FetchContent)
FetchContent_Declare(
    yamlcpp
    URL "https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz"
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
# Without this, yaml-cpp also configures util/yaml-cpp-parse, a standalone
# tool executable that links straight against system defaults (no
# -stdlib=libc++/static-runtime wiring) and fails to link on this box —
# confirmed the hard way. We only need the library.
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yamlcpp)

# stb_image.h / stb_image_write.h (Epic 1 Task 2 Step 5a): hoisted above both
# the tests/ and examples/ blocks below so test binaries (golden.cpp, PNG
# decode+encode) and examples/hello_frame.cpp (PNG encode) can both see them.
# Single-header public-domain libraries with no versioned release, so pinned
# to an immutable commit URL + sha256, same pattern as GetFilament.cmake.
set(STB_IMAGE_WRITE_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
set(STB_IMAGE_WRITE_SHA256 "cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05")
set(STB_IMAGE_COMMIT "${STB_IMAGE_WRITE_COMMIT}")  # same stb tree, both headers
set(STB_IMAGE_SHA256 "594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3")
set(STB_DIR "${CMAKE_BINARY_DIR}/_deps/stb")
set(STB_IMAGE_WRITE_H "${STB_DIR}/stb_image_write.h")
set(STB_IMAGE_H "${STB_DIR}/stb_image.h")
if(NOT EXISTS "${STB_IMAGE_WRITE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${STB_IMAGE_WRITE_COMMIT}/stb_image_write.h"
         "${STB_IMAGE_WRITE_H}" EXPECTED_HASH SHA256=${STB_IMAGE_WRITE_SHA256})
endif()
if(NOT EXISTS "${STB_IMAGE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${STB_IMAGE_COMMIT}/stb_image.h"
         "${STB_IMAGE_H}" EXPECTED_HASH SHA256=${STB_IMAGE_SHA256})
endif()

# Compile assets/materials/*.mat -> embeddable C++ headers via matc (shipped
# in the Filament SDK fetched above, ${FILAMENT_ROOT}/bin/matc) — see
# assets/materials/clay.mat's header comment for why a real material is
# needed at all (Engine::getDefaultMaterial() renders lighting-independent
# flat output, confirmed empirically in Epic 0/fixed in Task 2). Run once at
# configure time, like GetFilament.cmake's own file(DOWNLOAD): these .mat
# sources aren't expected to change mid-build, and this keeps the generated
# header available before target sources are even enumerated below.
set(_matc "${FILAMENT_ROOT}/bin/matc")
set(_generated_material_dir "${CMAKE_BINARY_DIR}/generated/visual_renderer_materials")
file(MAKE_DIRECTORY "${_generated_material_dir}")
file(GLOB _material_srcs "${CMAKE_CURRENT_SOURCE_DIR}/assets/materials/*.mat")
foreach(_mat_src ${_material_srcs})
    get_filename_component(_mat_name "${_mat_src}" NAME_WE)
    set(_mat_out "${_generated_material_dir}/${_mat_name}.filamat")
    set(_mat_header "${_generated_material_dir}/${_mat_name}_filamat.h")
    if("${_mat_src}" IS_NEWER_THAN "${_mat_header}")
        execute_process(
            COMMAND "${_matc}" -a opengl -p desktop -o "${_mat_out}" "${_mat_src}"
            RESULT_VARIABLE _matc_result
            ERROR_VARIABLE _matc_error)
        if(NOT _matc_result EQUAL 0)
            message(FATAL_ERROR "matc failed on ${_mat_src}: ${_matc_error}")
        endif()
        file(READ "${_mat_out}" _mat_hex HEX)
        string(REGEX REPLACE "(..)" "0x\\1," _mat_bytes "${_mat_hex}")
        file(WRITE "${_mat_header}"
            "// Generated by matc from assets/materials/${_mat_name}.mat. Do not edit.\n"
            "#pragma once\n"
            "#include <cstddef>\n"
            "#include <cstdint>\n"
            "namespace mpviz::materials {\n"
            "inline constexpr uint8_t k${_mat_name}Filamat[] = { ${_mat_bytes} };\n"
            "inline constexpr size_t k${_mat_name}FilamatSize = sizeof(k${_mat_name}Filamat);\n"
            "}  // namespace mpviz::materials\n")
    endif()
endforeach()

# GLOB with CONFIGURE_DEPENDS so later tasks (Epic 0 Task 2+) that add files
# under src/, tests/, examples/ don't need to touch this CMakeLists.txt too —
# a fresh `cmake` re-run picks them up.
file(GLOB_RECURSE VISUAL_RENDERER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(visual_renderer STATIC ${VISUAL_RENDERER_SOURCES})
add_library(micropilot_visualization::visual_renderer ALIAS visual_renderer)

target_include_directories(visual_renderer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# PRIVATE: matc-generated material headers are an internal renderer.cpp
# implementation detail, not part of the POD public API.
target_include_directories(visual_renderer PRIVATE "${CMAKE_BINARY_DIR}/generated/visual_renderer_materials")
# PRIVATE: Filament::filament's headers (and the raw static archives are
# still forwarded to any final executable regardless — that's how CMake
# handles static-lib link dependencies) must never leak to the gcc/
# libstdc++ ROS node that links this library — it only ever sees the POD
# api.h include/ dir above. See spec §2 / api.h's header comment.
target_link_libraries(visual_renderer PRIVATE
    Filament::filament
    yaml-cpp::yaml-cpp
    "${_libcxx_a}"
    "${_libcxxabi_a}"
    "${_libunwind_a}"
)

# Default theme-assets dir baked into the library, used by create_renderer()
# whenever RenderConfig::theme_assets_dir is null (Task 2 Step 5a/7).
target_compile_definitions(visual_renderer PRIVATE
    DEFAULT_THEME_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")

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
    # NOT find_package(GTest): this box's system libgtest.a (libgtest-dev) is
    # prebuilt against gcc/libstdc++, and linking it into a clang -stdlib=
    # libc++ test binary is exactly the cross-runtime ABI mix this directory
    # exists to avoid — confirmed by actually trying it first (Task 2):
    # every std:: symbol gtest-all.cc.o references (std::string, std::vector,
    # std::cerr, ...) came back undefined, because those are libstdc++
    # mangled names our libc++ link can't satisfy. Building GoogleTest from
    # source via FetchContent, using this same CMAKE_CXX_COMPILER/_FLAGS,
    # produces a libc++ gtest instead — same fix shape as GetFilament.cmake's
    # documented source-build fallback.
    include(FetchContent)
    set(GTEST_VERSION "1.15.2")
    FetchContent_Declare(
        googletest
        URL "https://github.com/google/googletest/archive/refs/tags/v${GTEST_VERSION}.tar.gz"
        URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926
    )
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # golden.cpp (Task 2 Step 9) is a shared render+compare helper, not a
    # gtest file itself — excluded from the auto-glob-as-gtest-binary loop
    # below by name, and instead compiled as a plain extra source into every
    # other test binary (same pattern as scene_buffer.cpp already is, just
    # explicit here since it lives under tests/ instead of src/).
    set(_golden_cpp "${CMAKE_CURRENT_SOURCE_DIR}/tests/golden.cpp")
    file(GLOB _visual_renderer_test_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    list(REMOVE_ITEM _visual_renderer_test_srcs "${_golden_cpp}")
    foreach(_test_src ${_visual_renderer_test_srcs})
        get_filename_component(_test_name "${_test_src}" NAME_WE)
        set(_test_extra_srcs "")
        if(EXISTS "${_golden_cpp}")
            set(_test_extra_srcs "${_golden_cpp}")
        endif()
        add_executable(${_test_name} "${_test_src}" ${_test_extra_srcs})
        # -I src: lets test TUs include internal-only headers like
        # scene_buffer.hpp (Task 1/VM-010). Deliberately NOT the Filament
        # include dir (that stays visual_renderer's PRIVATE-only) — internal
        # headers that pull in <filament/...> (e.g. renderer_internal.hpp)
        # are still off-limits to tests for that reason; see that header's
        # own comment.
        target_include_directories(${_test_name} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src" "${STB_DIR}")
        target_link_libraries(${_test_name} PRIVATE visual_renderer gtest gtest_main EGL)
        # MPVIZ_TEST_DATA_DIR: resolvable-path prefix for golden/fixture
        # literals (gtest_discover_tests' WORKING_DIRECTORY is the build
        # dir, not this source dir). MPVIZ_DEFAULT_THEME_DIR: kThemeDir's
        # backing macro (tests/test_paths.hpp) — Task 2 Step 5a.
        target_compile_definitions(${_test_name} PRIVATE
            MPVIZ_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
            MPVIZ_DEFAULT_THEME_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")
        include(GoogleTest)
        gtest_discover_tests(${_test_name} DISCOVERY_TIMEOUT 30)
    endforeach()
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    file(GLOB _visual_renderer_example_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach(_example_src ${_visual_renderer_example_srcs})
        get_filename_component(_example_name "${_example_src}" NAME_WE)
        add_executable(${_example_name} "${_example_src}")
        target_include_directories(${_example_name} PRIVATE "${STB_DIR}")
        target_link_libraries(${_example_name} PRIVATE visual_renderer)
    endforeach()
endif()
