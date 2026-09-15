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

# cesium-native (Epic 6, VM-061 — see
# docs/superpowers/plans/2026-08-18-visual-mode-epic6.md Task 2): pinned
# source build under this project's own clang/libc++ toolchain, its ~28
# vcpkg ports forced onto the same toolchain via the x64-linux-clang-libcxx
# overlay triplet. See cmake/GetCesiumNative.cmake for the pin + SHA256
# provenance and the vcpkg wiring.
include(cmake/GetCesiumNative.cmake)

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

# C++20 quarantine (Decision 3, VM-061/VM-062): environment_stream.cpp (Task
# 3 owns the file; it does not exist yet at VM-061) compiles in its own
# OBJECT library so it inherits cesium-native's FULL usage requirements --
# include dirs, PUBLIC compile definitions (GLM_FORCE_INTRINSICS et al.,
# configure_cesium_library.cmake:27-41 -- hand-copying only the include dirs
# would silently change glm's SIMD layout in this TU vs the archives), and
# the C++20 standard -- with nothing hand-copied to drift. Guarded by
# EXISTS so this task stays green standalone; Step 3's link probe is the
# compile proof that the cesium archive set itself is sound.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    list(REMOVE_ITEM VISUAL_RENDERER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
endif()

add_library(visual_renderer STATIC ${VISUAL_RENDERER_SOURCES})
add_library(micropilot_visualization::visual_renderer ALIAS visual_renderer)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    add_library(visual_renderer_stream OBJECT src/environment_stream.cpp)
    target_compile_features(visual_renderer_stream PRIVATE cxx_std_20)
    target_link_libraries(visual_renderer_stream PRIVATE
        Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync
        CesiumCurl   # the shipped curl network accessor (Decision 15.6, resolved);
                     # add CesiumUtility here too if the link demands it.
                     # CesiumIonClient deliberately ABSENT: the ion handshake is the
                     # Tileset ion ctor's job (Decision 15.6) -- nothing calls the
                     # REST client directly; re-add only if the link proves otherwise.
    )
    target_include_directories(visual_renderer_stream PRIVATE src include)
    target_sources(visual_renderer PRIVATE $<TARGET_OBJECTS:visual_renderer_stream>)
endif()
# Linked PRIVATE regardless of whether environment_stream.cpp exists yet
# (Task 3 lands the only TU that actually #includes cesium headers) --
# the OBJECT library above only compiles, it does not carry link
# dependencies into the final archive, so visual_renderer itself must still
# pull the cesium archives onto its own link line.
target_link_libraries(visual_renderer PRIVATE
    Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync CesiumCurl)

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

# Decision 4.3's named overlap risk, confirmed at link time (VM-061 Task 2
# Step 3/6): Filament's prebuilt SDK bundles its OWN draco (dracodec.a) and
# cesium-native pulls in vcpkg's OWN, separately-built draco (libdraco.a) --
# both land on every executable that links visual_renderer, both are
# libc++/std::__1 (same ABI family, unlike the YAML/spdlog/fmt case, so this
# is a plain duplicate-STRONG-symbol link error, not a silent-corruption
# ABI hazard), and GNU ld's ordinary link refuses with "multiple definition
# of draco::Options::GetString(...)" etc. INTERFACE so every consumer
# (filament_link_probe, cesium_link_probe, every tests/*.cpp binary) that
# links visual_renderer gets it too, since the collision only surfaces at
# the FINAL executable link, never in visual_renderer's own static-archive
# build. --allow-multiple-definition (first definition wins, matching link
# order) is the deliberate, narrower alternative to extending
# merge_yamlcpp.sh's rename recipe to draco too: draco's overlap here is
# recorded (Decision 4.3), not renamed, because unlike spdlog/fmt nothing
# reaches across this specific overlap from the gcc ROS node side -- both
# copies are internal to visual_renderer's own clang/libc++ link graph.
target_link_options(visual_renderer INTERFACE "-Wl,--allow-multiple-definition")

# Default theme-assets dir baked into the library, used by create_renderer()
# whenever RenderConfig::theme_assets_dir is null (Task 2 Step 5a/7).
target_compile_definitions(visual_renderer PRIVATE
    DEFAULT_THEME_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")

# Epic 2 Task 1 / VM-020 Step 0.2: ld -r visual_renderer's own objects and
# yaml-cpp's into one relocatable object (so theme.cpp's `YAML::` references
# resolve INSIDE it), then RENAME every remaining YAML::-namespaced defined
# symbol so it can never collide with the node's own, separately-built
# gcc/libstdc++ yaml-cpp (yaml_cpp_vendor) -- then rebuild libvisual_renderer.a
# FRESH from that merged+renamed object alone.
#
# --- THIRD recorded deviation from the plan's literal recipe (two earlier
# attempts -- merge+localize-by-visibility, then merge+localize-by-name --
# are recorded, with concrete failing evidence for each, in
# scripts/merge_yamlcpp.sh's header comment; read it before touching this
# again. Short version: localizing (changing binding, keeping the same
# name) is not enough -- the plan's own explicitly-sanctioned fallback of
# NOT localizing at all ("hidden-but-global ... link order decides") was
# also tried and empirically corrupted node-side YAML parsing (right error
# count, wrong row count, then a segfault) the first time real node code
# called the same yaml-cpp template accessor this archive's theme.cpp also
# calls. The fix that actually holds is renaming this archive's copy so the
# two never share a symbol name at all, which scripts/merge_yamlcpp.sh now
# does via `objcopy --redefine-syms` in place of `--localize-symbols`/
# `--localize-hidden`. ---
# --- 2026-09-15 extension (Epic 6 / VM-061 Task 2 Step 5, Decision 4):
# collect every cesium-native module archive + spdlog (+ fmt, if separate)
# reachable from the targets visual_renderer links, so merge_yamlcpp.sh can
# `ld -r` + rename them alongside yaml-cpp -- see that script's own header
# comment for why (the ROS node's gcc/libstdc++ spdlog vs. cesium's
# clang/libc++ one, Decision 4). Walks INTERFACE_LINK_LIBRARIES/
# LINK_LIBRARIES breadth-first but only RECURSES PAST (and collects the
# archive of) a target whose short name (namespace prefix stripped) is
# "Cesium*", "spdlog", or "fmt" -- the C/C++ leaf dependencies Decision 4.2/
# 4.3 deliberately leaves merged-in-place and AUDITED instead (curl,
# openssl, sqlite3, draco, ktx, meshoptimizer, s2geometry, ...) are neither
# recursed into nor added here.
function(_mpviz_vendor_merge_target_p _tgt _out)
    string(REGEX REPLACE "^.*::" "" _short "${_tgt}")
    if(_short MATCHES "^Cesium" OR _short STREQUAL "spdlog" OR _short STREQUAL "fmt")
        set(${_out} TRUE PARENT_SCOPE)
    else()
        set(${_out} FALSE PARENT_SCOPE)
    endif()
endfunction()

macro(_mpviz_collect_vendor_merge_archives _out_var)
    set(_mpviz_cma_result "")
    set(_mpviz_cma_seen "")
    set(_mpviz_cma_queue ${ARGN})
    while(_mpviz_cma_queue)
        list(POP_FRONT _mpviz_cma_queue _mpviz_cma_cur)
        if(NOT _mpviz_cma_cur IN_LIST _mpviz_cma_seen)
            list(APPEND _mpviz_cma_seen "${_mpviz_cma_cur}")
            if(TARGET "${_mpviz_cma_cur}")
                _mpviz_vendor_merge_target_p("${_mpviz_cma_cur}" _mpviz_cma_is_vendor)
                if(_mpviz_cma_is_vendor)
                    get_target_property(_mpviz_cma_type "${_mpviz_cma_cur}" TYPE)
                    if(_mpviz_cma_type STREQUAL "STATIC_LIBRARY")
                        list(APPEND _mpviz_cma_result "$<TARGET_FILE:${_mpviz_cma_cur}>")
                    endif()
                    foreach(_mpviz_cma_prop IN ITEMS INTERFACE_LINK_LIBRARIES LINK_LIBRARIES)
                        get_target_property(_mpviz_cma_deps "${_mpviz_cma_cur}" ${_mpviz_cma_prop})
                        if(_mpviz_cma_deps)
                            foreach(_mpviz_cma_dep ${_mpviz_cma_deps})
                                string(REGEX REPLACE "^\\$<LINK_ONLY:(.*)>$" "\\1" _mpviz_cma_dep_clean "${_mpviz_cma_dep}")
                                list(APPEND _mpviz_cma_queue "${_mpviz_cma_dep_clean}")
                            endforeach()
                        endif()
                    endforeach()
                endif()
            endif()
        endif()
    endwhile()
    set(${_out_var} "${_mpviz_cma_result}")
endmacro()

_mpviz_collect_vendor_merge_archives(_mpviz_vendor_merge_archives
    Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync CesiumCurl)
list(REMOVE_DUPLICATES _mpviz_vendor_merge_archives)
if(NOT _mpviz_vendor_merge_archives)
    message(FATAL_ERROR
        "visual_renderer: the cesium-native/spdlog vendor-merge archive walk "
        "(VM-061 Step 5) found nothing -- either cesium's target names "
        "changed at this pin, or the walk itself is broken. Never silently "
        "skip the symbol-hygiene merge; fix the walk or the target list.")
endif()

add_custom_command(TARGET visual_renderer POST_BUILD
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/merge_yamlcpp.sh"
            "${CMAKE_AR}" "${CMAKE_LINKER}" "${CMAKE_OBJCOPY}" "${CMAKE_NM}"
            "${CMAKE_BINARY_DIR}/_yamlmerge"
            "${CMAKE_BINARY_DIR}/visual_renderer_merged.o"
            "$<TARGET_FILE:visual_renderer>"
            $<TARGET_OBJECTS:visual_renderer>
            --
            "$<TARGET_FILE:yaml-cpp>"
            ${_mpviz_vendor_merge_archives}
    COMMAND_EXPAND_LISTS VERBATIM
    COMMENT "visual_renderer: merging yaml-cpp + cesium-native + spdlog/fmt, renaming their vendored symbols (Epic 2 Task 1 Step 0.2; extended Epic 6 VM-061 Step 5)")

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

# Build-integration proof for cesium-native (VM-061 Task 2 Step 3), same
# purpose as filament_link_probe above: prove the archive set actually
# LINKS and RUNS, since a static-lib build alone never invokes the linker.
# C++20 TU (cesium headers require it). Links visual_renderer PRIVATE too
# (not just the two cesium targets it #includes directly) so this probe
# exercises the SAME combined archive graph (Filament + yaml-cpp + cesium)
# Task 3's real environment_stream.cpp will pull in through visual_renderer
# -- the cyclic-archive-handling AC (B06.2.1) this step maps to. Unlike
# Filament's raw .a-file target, cesium's CMake targets (CesiumGeospatial,
# CesiumGltfReader) are real targets with their own PUBLIC include dirs, so
# linking them directly is what "grants cesium includes to the probe" here
# -- no manual -I needed the way FILAMENT_ROOT/include was for Filament.
add_executable(cesium_link_probe smoke/cesium_link_probe.cpp)
set_target_properties(cesium_link_probe PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES)
target_link_libraries(cesium_link_probe PRIVATE
    visual_renderer CesiumGeospatial CesiumGltfReader)
add_test(NAME cesium_link_probe COMMAND cesium_link_probe)

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

# tools/ — standalone binaries linking visual_renderer, same glob pattern as
# examples/ above (VM-041, Epic 5: viz_benchmark.cpp prints render_ms
# p50/p99 per quality preset for VM-040's governor to tune against later).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    file(GLOB _visual_renderer_tool_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/*.cpp")
    foreach(_tool_src ${_visual_renderer_tool_srcs})
        get_filename_component(_tool_name "${_tool_src}" NAME_WE)
        add_executable(${_tool_name} "${_tool_src}")
        target_link_libraries(${_tool_name} PRIVATE visual_renderer)
    endforeach()
endif()
