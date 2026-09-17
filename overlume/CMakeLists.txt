cmake_minimum_required(VERSION 3.18)

# overlume is its own CMake project (not add_subdirectory'd into the
# gcc-toolchain `ros/` colcon build): the toolchain boundary in the design spec
# (docs/design/2026-08-18-visual-mode-design.md §2) requires this
# library to compile clang/libc++ while the rest of the CUDA/ROS stack stays
# on gcc/libstdc++ — CMAKE_CXX_COMPILER is fixed per top-level project, so the
# split has to happen at this granularity. Consumers (the ROS node) link
# against it only through the POD-only public header in include/.
project(overlume LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Configure with `cmake --toolchain cmake/toolchain-clang-libcxx.cmake` (see
# that file + scripts/setup_toolchain_cesium.sh + README.md) — it selects a working
# clang++/libc++ and bakes in -stdlib=libc++. These are defensive checks in
# case someone configures by hand instead.
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR
        "overlume must be built with clang++ (got CMAKE_CXX_COMPILER_ID="
        "'${CMAKE_CXX_COMPILER_ID}'). Configure with:\n"
        "  cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S . \n"
        "(run scripts/setup_toolchain_cesium.sh first if you have no system clang++/libc++). "
        "See README.md.")
endif()
if(NOT CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    message(FATAL_ERROR
        "overlume must be built against libc++ (pass "
        "-DCMAKE_CXX_FLAGS=-stdlib=libc++), which the toolchain file above "
        "already does. See spec §2 for why gcc/libstdc++ and clang/libc++ "
        "must not mix.")
endif()

# overlume links libc++/libc++abi/libunwind as explicit static
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
        "overlume: could not resolve a static libc++.a from compiler "
        "'${CMAKE_CXX_COMPILER}' (looked at '${_libcxx_a}'). Is "
        "CMAKE_CXX_COMPILER a real clang++ with a co-located libc++-dev?")
endif()
get_filename_component(_libcxx_lib_dir "${_libcxx_a}" DIRECTORY)
set(_libcxxabi_a "${_libcxx_lib_dir}/libc++abi.a")
set(_libunwind_a "${_libcxx_lib_dir}/libunwind.a")
foreach(_rt_lib "${_libcxxabi_a}" "${_libunwind_a}")
    if(NOT EXISTS "${_rt_lib}")
        message(FATAL_ERROR
            "overlume: expected static runtime lib not found: "
            "${_rt_lib} (sibling of ${_libcxx_a}). Toolchain layout mismatch?")
    endif()
endforeach()

include(cmake/GetFilament.cmake)

# cesium-native (Epic 6, VM-061 — see
# docs/plans/2026-08-18-visual-mode-epic6.md Task 2): pinned
# source build under this project's own clang/libc++ toolchain, its ~28
# vcpkg ports forced onto the same toolchain via the x64-linux-clang-libcxx
# overlay triplet. See cmake/GetCesiumNative.cmake for the pin + SHA256
# provenance and the vcpkg wiring.
#
# VM-061 gate round 2, Finding 3 (ORIGINAL blocker, RESOLVED at Step 6 —
# USER DECISION a, 2026-09-15): this project's primary toolchain was
# clang-14 (toolchain-clang-libcxx.cmake / scripts/setup_toolchain.sh, what
# ci_visual_mode.sh/colcon_build.sh configure `build/` with), which cannot
# compile cesium-native's vcpkg dependency ada-url -- its
# url_search_params-inl.h calls std::ranges::replace, a <ranges> algorithm
# overload libc++-14 does not implement (reproduced directly: a trivial
# std::ranges::replace program fails to compile under -stdlib=libc++
# -std=gnu++20 with clang-14, "no member named 'replace' in namespace
# 'std::ranges'"). Cesium-native was only buildable under the SEPARATE
# clang-18 toolchain scripts/setup_toolchain_cesium.sh bootstraps (see that
# script's header for the full story + why clang-18 specifically).
#
# RESOLVED: the primary toolchain itself (toolchain-clang-libcxx.cmake) now
# resolves to that same clang-18 prefix (option (a) of the two named — (b),
# an ExternalProject_Add nested build keeping the primary at clang-14, was
# REJECTED by the user as an unwarranted rewrite). So `build/` and
# `build-cesium/` are now the SAME compiler; this option still gates every
# cesium-touching piece below (this include, the overlume_stream
# C++20 OBJECT library, the cesium target_link_libraries/link_options on
# overlume, the vendor-merge archive walk, and cesium_link_probe) —
# default stays OFF so an ordinary `build/` configure (no `-D` override)
# keeps behaving exactly as it did pre-Epic-6; pass -DOVERLUME_ENABLE_CESIUM=ON
# to build the cesium path (no longer needs a different toolchain to do so).
option(OVERLUME_ENABLE_CESIUM
    "Build the cesium-native path (Epic 6 / VM-061). Requires the (now \
primary) clang-18 toolchain (scripts/setup_toolchain_cesium.sh). OFF keeps \
the ordinary build/ tree buildable without ever touching cesium-native." OFF)
if(OVERLUME_ENABLE_CESIUM)
    include(cmake/GetCesiumNative.cmake)
endif()

# yaml-cpp (Epic 1 Task 2 / VM-011): theme.cpp parses assets/themes/*.yaml.
# FetchContent'd and built with this project's own clang/libc++ toolchain for
# the same ABI-safety reason as GoogleTest below (a prebuilt system
# libyaml-cpp would be gcc/libstdc++). Linked PRIVATE into overlume —
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
# decode+encode) and the repo-root examples (PNG encode) can both see them.
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
set(_generated_material_dir "${CMAKE_BINARY_DIR}/generated/overlume_materials")
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
            "namespace overlume::materials {\n"
            "inline constexpr uint8_t k${_mat_name}Filamat[] = { ${_mat_bytes} };\n"
            "inline constexpr size_t k${_mat_name}FilamatSize = sizeof(k${_mat_name}Filamat);\n"
            "}  // namespace overlume::materials\n")
    endif()
endforeach()

# GLOB with CONFIGURE_DEPENDS so later tasks (Epic 0 Task 2+) that add files
# under src/, tests/, examples/ don't need to touch this CMakeLists.txt too —
# a fresh `cmake` re-run picks them up.
file(GLOB_RECURSE OVERLUME_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# C++20 quarantine (Decision 3, VM-061/VM-062): environment_stream.cpp (Task
# 3's file, now landed) compiles in its own OBJECT library so it inherits
# cesium-native's FULL usage requirements -- include dirs, PUBLIC compile
# definitions (GLM_FORCE_INTRINSICS et al., configure_cesium_library.cmake:
# 27-41 -- hand-copying only the include dirs would silently change glm's
# SIMD layout in this TU vs the archives), and the C++20 standard -- with
# nothing hand-copied to drift.
#
# VM-062 fix (this task): the EXISTS-only removal below is UNCONDITIONAL on
# OVERLUME_ENABLE_CESIUM, deliberately -- with the option OFF, environment_stream.cpp
# must be excluded from the ordinary C++17 glob entirely (not compiled at
# all, by either path) rather than fall through into OVERLUME_SOURCES
# and get compiled as a plain C++17 TU: the file's own
# static_assert(__cplusplus >= 202002L, ...) (environment_stream.hpp) would
# hard-fail that build, which is exactly the "your changes must be invisible
# in build/" hard rule this file's own header comment already promises.
# Only the OBJECT-library block two lines down stays gated on
# OVERLUME_ENABLE_CESIUM -- that's what actually compiles the file.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    list(REMOVE_ITEM OVERLUME_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
endif()

add_library(overlume STATIC ${OVERLUME_SOURCES})
add_library(overlume::overlume ALIAS overlume)

if(OVERLUME_ENABLE_CESIUM AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    add_library(overlume_stream OBJECT src/environment_stream.cpp)
    target_compile_features(overlume_stream PRIVATE cxx_std_20)
    target_link_libraries(overlume_stream PRIVATE
        Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync
        CesiumCurl   # the shipped curl network accessor (Decision 15.6, resolved);
                     # add CesiumUtility here too if the link demands it.
                     # CesiumIonClient deliberately ABSENT: the ion handshake is the
                     # Tileset ion ctor's job (Decision 15.6) -- nothing calls the
                     # REST client directly; re-add only if the link proves otherwise.
        # VM-062: environment_stream.cpp includes renderer_internal.hpp +
        # environment.hpp, both of which pull in <filament/...>/<gltfio/...>
        # headers (the createAsset/loadResources/TransformManager/
        # RenderableManager calls Decision 7/8 need) -- OBJECT libraries
        # inherit usage requirements (include dirs) from what they link,
        # same mechanism this file's own Decision-3 comment relies on for
        # cesium's PUBLIC compile definitions, so this grants headers only,
        # no extra link dependency (overlume already links
        # Filament::filament itself).
        Filament::filament
    )
    target_include_directories(overlume_stream PRIVATE src include)
    target_sources(overlume PRIVATE $<TARGET_OBJECTS:overlume_stream>)
endif()
# Linked PRIVATE regardless of whether environment_stream.cpp exists yet
# (Task 3 lands the only TU that actually #includes cesium headers) --
# the OBJECT library above only compiles, it does not carry link
# dependencies into the final archive, so overlume itself must still
# pull the cesium archives onto its own link line.
#
# VM-061 gate round 2, Finding 2: linked exactly as before ($<LINK_ONLY:>
# wrapping was tried FIRST and empirically does NOT work -- reproduced with a
# standalone 4-line CMake repro that CMake still bumps a STATIC library's OWN
# required C++ standard to match a linked dependency's PUBLIC/INTERFACE
# compile features regardless of PRIVATE/$<LINK_ONLY:> on OUR end; that
# generator expression only strips usage requirements forwarded to
# overlume's CONSUMERS, it does nothing for overlume's OWN
# compile. A second attempt -- linking the raw resolved archive FILES
# ($<TARGET_FILE:...>) instead of the CMake target names, which DOES avoid
# the propagation -- was also tried and reverted: it forfeits CMake's own
# link-line ordering/cycle handling for cesium's ~19-archive + spdlog/fmt/
# draco/curl/absl/zlib/... closure, and even wrapped in
# -Wl,--start-group/--end-group it broke unrelated, previously-working links
# (Filament's own TangentSpaceMesh symbols, curl's zlib symbols) that aren't
# this project's to debug as part of a C++-standard quarantine fix. What
# actually holds, with NO change to how cesium is linked or ordered: force
# the EFFECTIVE standard back down with an explicit, LATER `-std=` flag --
# both gcc and clang resolve duplicate -std= flags by "last one wins" (
# confirmed directly against this project's own clang++ wrapper: `-std=
# gnu++20 -std=gnu++17` compiles as __cplusplus == 201703L), and CMake places
# a target's own COMPILE_OPTIONS after the auto-computed standard flag in
# CXX_FLAGS, so this one line reliably wins over whatever cesium's compile
# features bump the computed standard to.
if(OVERLUME_ENABLE_CESIUM)
    # Only needed when cesium's C++20 compile features are in the graph; derived
    # from the project's own standard so a future bump carries through instead of
    # being silently clamped back.
    target_compile_options(overlume PRIVATE -std=gnu++${CMAKE_CXX_STANDARD})
    # VM-062: lets environment.cpp's ion:// dispatch (Decision 5) call
    # open_streaming_environment_source() only when environment_stream.cpp
    # was actually compiled in -- with the option OFF, that symbol has no
    # definition anywhere in this build, so the call site is compiled out
    # instead of becoming an undefined reference at every executable that
    # links overlume (hello_frame, every tests/*.cpp binary, ...).
    target_compile_definitions(overlume PRIVATE OVERLUME_ENABLE_CESIUM=1)
endif()

target_include_directories(overlume PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# PRIVATE: matc-generated material headers are an internal renderer.cpp
# implementation detail, not part of the POD public API.
target_include_directories(overlume PRIVATE "${CMAKE_BINARY_DIR}/generated/overlume_materials")
# PRIVATE: Filament::filament's headers (and the raw static archives are
# still forwarded to any final executable regardless — that's how CMake
# handles static-lib link dependencies) must never leak to the gcc/
# libstdc++ ROS node that links this library — it only ever sees the POD
# api.h include/ dir above. See spec §2 / api.h's header comment.
# VM-061 Step 6 (2026-09-15): when OVERLUME_ENABLE_CESIUM is ON, the static
# runtime archives below are opened in a "-Wl,--start-group" here and closed
# with a matching "-Wl,--end-group" AFTER cesium's own archives are linked
# (below, once OVERLUME_ENABLE_CESIUM's target_link_libraries calls have run) --
# see that closing marker's own comment for the full root-cause story
# (cesium's vcpkg debug archives need a libc++ symbol that's positioned too
# early otherwise). Gated on OVERLUME_ENABLE_CESIUM so the OFF path's link line
# is BYTE-IDENTICAL to before this fix (proven green, untouched).
#
# Tried and reverted first: repeating the SAME three archive paths a second
# time (self-wrapped in their own start/end group) after the cesium links,
# instead of wrapping the ORIGINAL single occurrence. This does NOT work --
# confirmed directly against a generated link.txt: CMake's link-line
# computation treats identical repeated raw-path items across separate
# target_link_libraries calls as ONE logical entry and collapses them back
# together at the FIRST call's position, while the (non-duplicated, so never
# collapsed) "-Wl,--start-group"/"-Wl,--end-group" marker strings stayed at
# the SECOND call's position -- producing an EMPTY group at the very end of
# the command line and the original ungrouped triple sitting right where it
# always was, so the fix silently did nothing. Wrapping the one, singular
# occurrence (this block) and closing the group only after everything that
# might need it has been linked is what actually works with CMake's
# behavior instead of fighting it.
if(OVERLUME_ENABLE_CESIUM)
    target_link_libraries(overlume PRIVATE "-Wl,--start-group")
endif()
target_link_libraries(overlume PRIVATE
    Filament::filament
    yaml-cpp::yaml-cpp
    "${_libcxx_a}"
    "${_libcxxabi_a}"
    "${_libunwind_a}"
)

# Decision 4.3's named overlap risk, confirmed at link time (VM-061 Task 2
# Step 3/6): Filament's prebuilt SDK bundles its OWN draco (dracodec.a) and
# cesium-native pulls in vcpkg's OWN, separately-built draco (libdraco.a) --
# both land on every executable that links overlume, both are
# libc++/std::__1 (same ABI family, unlike the YAML/spdlog/fmt case, so this
# is a plain duplicate-STRONG-symbol link error, not a silent-corruption
# ABI hazard), and GNU ld's ordinary link refuses with "multiple definition
# of draco::Options::GetString(...)" etc. INTERFACE so every consumer
# (filament_link_probe, cesium_link_probe, every tests/*.cpp binary) that
# links overlume gets it too, since the collision only surfaces at
# the FINAL executable link, never in overlume's own static-archive
# build. --allow-multiple-definition (first definition wins, matching link
# order) is the deliberate, narrower alternative to extending
# merge_yamlcpp.sh's rename recipe to draco too: draco's overlap here is
# recorded (Decision 4.3), not renamed, because unlike spdlog/fmt nothing
# reaches across this specific overlap from the gcc ROS node side -- both
# copies are internal to overlume's own clang/libc++ link graph.
if(OVERLUME_ENABLE_CESIUM)
    target_link_options(overlume INTERFACE "-Wl,--allow-multiple-definition")
endif()

# Default theme-assets dir baked into the library, used by create_renderer()
# whenever RenderConfig::theme_assets_dir is null (Task 2 Step 5a/7).
target_compile_definitions(overlume PRIVATE
    DEFAULT_THEME_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")

# Epic 2 Task 1 / VM-020 Step 0.2: ld -r overlume's own objects and
# yaml-cpp's into one relocatable object (so theme.cpp's `YAML::` references
# resolve INSIDE it), then RENAME every remaining YAML::-namespaced defined
# symbol so it can never collide with the node's own, separately-built
# gcc/libstdc++ yaml-cpp (yaml_cpp_vendor) -- then rebuild liboverlume.a
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
# reachable from the targets overlume links, so merge_yamlcpp.sh can
# `ld -r` + rename them alongside yaml-cpp -- see that script's own header
# comment for why (the ROS node's gcc/libstdc++ spdlog vs. cesium's
# clang/libc++ one, Decision 4). Walks INTERFACE_LINK_LIBRARIES/
# LINK_LIBRARIES breadth-first but only RECURSES PAST (and collects the
# archive of) a target whose short name (namespace prefix stripped) is
# "Cesium*", "spdlog", or "fmt" -- the C/C++ leaf dependencies Decision 4.2/
# 4.3 deliberately leaves merged-in-place and AUDITED instead (curl,
# openssl, sqlite3, draco, ktx, meshoptimizer, s2geometry, ...) are neither
# recursed into nor added here.
function(_overlume_vendor_merge_target_p _tgt _out)
    string(REGEX REPLACE "^.*::" "" _short "${_tgt}")
    if(_short MATCHES "^Cesium" OR _short STREQUAL "spdlog" OR _short STREQUAL "fmt")
        set(${_out} TRUE PARENT_SCOPE)
    else()
        set(${_out} FALSE PARENT_SCOPE)
    endif()
endfunction()

macro(_overlume_collect_vendor_merge_archives _out_var)
    set(_overlume_cma_result "")
    set(_overlume_cma_seen "")
    set(_overlume_cma_queue ${ARGN})
    while(_overlume_cma_queue)
        list(POP_FRONT _overlume_cma_queue _overlume_cma_cur)
        if(NOT _overlume_cma_cur IN_LIST _overlume_cma_seen)
            list(APPEND _overlume_cma_seen "${_overlume_cma_cur}")
            if(TARGET "${_overlume_cma_cur}")
                _overlume_vendor_merge_target_p("${_overlume_cma_cur}" _overlume_cma_is_vendor)
                if(_overlume_cma_is_vendor)
                    get_target_property(_overlume_cma_type "${_overlume_cma_cur}" TYPE)
                    if(_overlume_cma_type STREQUAL "STATIC_LIBRARY")
                        list(APPEND _overlume_cma_result "$<TARGET_FILE:${_overlume_cma_cur}>")
                    endif()
                    foreach(_overlume_cma_prop IN ITEMS INTERFACE_LINK_LIBRARIES LINK_LIBRARIES)
                        get_target_property(_overlume_cma_deps "${_overlume_cma_cur}" ${_overlume_cma_prop})
                        if(_overlume_cma_deps)
                            foreach(_overlume_cma_dep ${_overlume_cma_deps})
                                string(REGEX REPLACE "^\\$<LINK_ONLY:(.*)>$" "\\1" _overlume_cma_dep_clean "${_overlume_cma_dep}")
                                list(APPEND _overlume_cma_queue "${_overlume_cma_dep_clean}")
                            endforeach()
                        endif()
                    endforeach()
                endif()
            endif()
        endif()
    endwhile()
    set(${_out_var} "${_overlume_cma_result}")
endmacro()

if(OVERLUME_ENABLE_CESIUM)
    set(_overlume_vendor_merge_direct_targets
        Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync CesiumCurl)

    # Finding 2 is handled by the `target_compile_options(overlume
    # PRIVATE -std=gnu++17)` line above, not here -- this ordinary
    # target-name link is exactly what round 1 shipped (needed as-is so
    # CMake's own link-line ordering/cycle handling for cesium's ~19-archive
    # + spdlog/fmt/draco/curl/absl/zlib/... closure keeps working; see that
    # line's comment for the two approaches that were tried instead and
    # reverted).
    target_link_libraries(overlume PRIVATE ${_overlume_vendor_merge_direct_targets})
    _overlume_collect_vendor_merge_archives(_overlume_vendor_merge_archives
        ${_overlume_vendor_merge_direct_targets})
    list(REMOVE_DUPLICATES _overlume_vendor_merge_archives)

    # VM-061 Step 6 (2026-09-15): finding, root-caused directly against a
    # generated link.txt. cesium's own transitive closure carries vcpkg's
    # DEBUG-build libktx.a (Decision 4.3: ktx is "recorded, not renamed" --
    # it stays a plain standalone archive on the link line, unlike the
    # spdlog/fmt/Cesium* set that gets ld -r-merged above) whose
    # basisu_transcoder.cpp.o references std::__1::__sort<...> (a libc++
    # extern-template instantiation that DOES exist, as a weak symbol, in
    # libc++.a). But libc++.a/libc++abi.a/libunwind.a are linked ABOVE
    # (Filament::filament / yaml-cpp::yaml-cpp's own target_link_libraries
    # call), i.e. BEFORE this cesium block on the final executable's link
    # line -- confirmed directly: build/CMakeFiles/cesium_link_probe.dir/
    # link.txt lists libc++.a at position 51, libktx.a at position 145.
    # GNU ld's single-pass archive resolution processes archives strictly
    # left-to-right, so a later archive's need for an earlier archive's
    # symbol is never satisfied -- "undefined reference to
    # std::__1::__sort<...>" at link time for every executable that links
    # overlume. This never surfaced in build-cesium/ because that
    # tree's OWN project toolchain (vcpkg-clang-libcxx-toolchain.cmake) links
    # libc++ DYNAMICALLY, where position doesn't gate archive-member
    # extraction the same way -- a genuinely new failure mode from combining
    # cesium's vcpkg debug archives with the primary toolchain's static/
    # -nostdlib++ convention (VM-061 Step 6, first time these two ever
    # combined). Fix: the ORIGINAL libc++.a/libc++abi.a/libunwind.a
    # target_link_libraries call (above) opens a "-Wl,--start-group" right
    # before them when this option is ON; closed here, once every
    # OVERLUME_ENABLE_CESIUM link addition (this whole block, including its
    # transitive ktx/draco/spdlog/fmt/curl/... closure) has actually been
    # appended -- so GNU ld repeatedly re-scans the WHOLE bracketed region
    # (runtime archives + the entire cesium closure) until every symbol
    # inside it is resolved, regardless of which side references which.
    #
    # Tried and reverted first: repeating the three runtime archive paths a
    # SECOND time here, self-wrapped in their own start/end group, instead
    # of wrapping the original single occurrence. Does NOT work -- confirmed
    # directly against a generated link.txt: CMake's link-line computation
    # treats identical repeated raw-path items across separate
    # target_link_libraries calls as ONE logical entry and collapses them
    # back together at the FIRST call's position, while the (never-repeated,
    # so never collapsed) "-Wl,--start-group"/"-Wl,--end-group" strings
    # stayed at the SECOND call's position -- producing an EMPTY group at
    # the very end of the command line and the original ungrouped triple
    # sitting exactly where it always was; the fix silently did nothing.
    target_link_libraries(overlume PRIVATE "-Wl,--end-group")

    # VM-061 gate round 2, Finding 1: an empty-list check alone already missed a
    # real failure once (spdlog::spdlog/fmt::fmt are IMPORTED, non-GLOBAL targets
    # -- find_package(spdlog CONFIG REQUIRED) runs deep inside cesium-native's own
    # FetchContent'd directory scope, so the resulting imported target is visible
    # only there and in its subdirectories, NOT back up here at the top level --
    # `if(TARGET spdlog::spdlog)` silently returned FALSE at THIS scope even
    # though the target existed, so the walk collected all ~19 Cesium* archives
    # and quietly dropped spdlog/fmt with no error at all). Root-caused and fixed
    # at the source in cmake/GetCesiumNative.cmake (CMAKE_FIND_PACKAGE_TARGETS_GLOBAL);
    # this guard is the second, independent line of defense: require at least one
    # spdlog archive AND at least as many Cesium archives as targets directly
    # linked, so a similar silent partial-miss (this or any future cesium
    # version) fails loudly instead of shipping a library that looks merged but
    # isn't.
    set(_overlume_spdlog_archive_count 0)
    set(_overlume_cesium_archive_count 0)
    foreach(_overlume_cma_archive ${_overlume_vendor_merge_archives})
        if(_overlume_cma_archive MATCHES "[Ss]pdlog")
            math(EXPR _overlume_spdlog_archive_count "${_overlume_spdlog_archive_count} + 1")
        endif()
        if(_overlume_cma_archive MATCHES "Cesium")
            math(EXPR _overlume_cesium_archive_count "${_overlume_cesium_archive_count} + 1")
        endif()
    endforeach()
    list(LENGTH _overlume_vendor_merge_direct_targets _overlume_vendor_merge_direct_target_count)
    if(_overlume_spdlog_archive_count LESS 1 OR _overlume_cesium_archive_count LESS _overlume_vendor_merge_direct_target_count)
        message(FATAL_ERROR
            "overlume: the cesium-native/spdlog vendor-merge archive walk "
            "(VM-061 Step 5) is unsound -- collected ${_overlume_cesium_archive_count} "
            "Cesium* archives (need >= ${_overlume_vendor_merge_direct_target_count}, "
            "one reachable per directly-linked target: ${_overlume_vendor_merge_direct_targets}) "
            "and ${_overlume_spdlog_archive_count} spdlog archive(s) (need >= 1). "
            "Either cesium's target names/link graph changed at this pin, or the "
            "walk itself is broken again. Never silently skip the symbol-hygiene "
            "merge; fix the walk or the target list.")
    endif()
else()
    set(_overlume_vendor_merge_archives "")
endif()

# VM-062 fix: $<TARGET_OBJECTS:overlume> does NOT include
# environment_stream.cpp.o even though it's added to overlume's own
# SOURCES via target_sources(... $<TARGET_OBJECTS:overlume_stream>)
# above -- confirmed directly (generated build.make's merge command line
# lists every ordinary tests/*.cpp-glob .o file but not this one), a CMake
# generator-expression quirk: TARGET_OBJECTS enumerates a target's objects
# compiled from its own literal SOURCES list, not objects forwarded into it
# through a nested TARGET_OBJECTS generator expression. Symptom this caused
# before the fix: `open_streaming_environment_source` compiled fine
# (overlume_stream.cpp.o has the DEFINED symbol, verified with nm)
# but never made it into overlume_merged.o, so every executable
# that links overlume failed with an undefined reference the moment
# environment.cpp's ion:// dispatch called it. Fixed by passing
# overlume_stream's own object list to the merge script explicitly,
# alongside overlume's.
set(_overlume_overlume_stream_objects "")
if(OVERLUME_ENABLE_CESIUM AND TARGET overlume_stream)
    set(_overlume_overlume_stream_objects $<TARGET_OBJECTS:overlume_stream>)
endif()

add_custom_command(TARGET overlume POST_BUILD
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/merge_yamlcpp.sh"
            "${CMAKE_AR}" "${CMAKE_LINKER}" "${CMAKE_OBJCOPY}" "${CMAKE_NM}"
            "${CMAKE_BINARY_DIR}/_yamlmerge"
            "${CMAKE_BINARY_DIR}/overlume_merged.o"
            "$<TARGET_FILE:overlume>"
            $<TARGET_OBJECTS:overlume>
            ${_overlume_overlume_stream_objects}
            --
            "$<TARGET_FILE:yaml-cpp>"
            ${_overlume_vendor_merge_archives}
    COMMAND_EXPAND_LISTS VERBATIM
    COMMENT "overlume: merging yaml-cpp + cesium-native + spdlog/fmt, renaming their vendored symbols (Epic 2 Task 1 Step 0.2; extended Epic 6 VM-061 Step 5)")

enable_testing()

add_test(NAME check_pod_header
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_pod_header.sh)

# Build-integration proof (Epic 0 Task 1): actually links `overlume`
# (and therefore the ~30 cross-referencing Filament static archives pulled in
# by cmake/GetFilament.cmake) into a real executable and exercises a call
# that spans that archive graph. check_pod_header.sh proves the header is
# POD-clean; this proves the link recipe itself is sound — neither the
# static-lib-only build above nor the header check ever invokes the linker
# against Filament's archives. See smoke/filament_link_probe.cpp.
add_executable(filament_link_probe smoke/filament_link_probe.cpp)
# This probe deliberately calls Filament directly (see file header comment)
# to prove the archive-link-order fix, so it needs Filament's headers itself
# — granted explicitly here, NOT inherited from overlume (whose
# Filament dependency is PRIVATE precisely so ordinary consumers, i.e. the
# eventual gcc ROS node, never see them).
target_include_directories(filament_link_probe PRIVATE "${FILAMENT_ROOT}/include")
target_link_libraries(filament_link_probe PRIVATE overlume)
add_test(NAME filament_link_probe COMMAND filament_link_probe)

# Build-integration proof for cesium-native (VM-061 Task 2 Step 3), same
# purpose as filament_link_probe above: prove the archive set actually
# LINKS and RUNS, since a static-lib build alone never invokes the linker.
# C++20 TU (cesium headers require it). Links overlume PRIVATE too
# (not just the two cesium targets it #includes directly) so this probe
# exercises the SAME combined archive graph (Filament + yaml-cpp + cesium)
# Task 3's real environment_stream.cpp will pull in through overlume
# -- the cyclic-archive-handling AC (B06.2.1) this step maps to. Unlike
# Filament's raw .a-file target, cesium's CMake targets (CesiumGeospatial,
# CesiumGltfReader) are real targets with their own PUBLIC include dirs, so
# linking them directly is what "grants cesium includes to the probe" here
# -- no manual -I needed the way FILAMENT_ROOT/include was for Filament.
if(OVERLUME_ENABLE_CESIUM)
    add_executable(cesium_link_probe smoke/cesium_link_probe.cpp)
    set_target_properties(cesium_link_probe PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES)
    target_link_libraries(cesium_link_probe PRIVATE
        overlume CesiumGeospatial CesiumGltfReader)
    add_test(NAME cesium_link_probe COMMAND cesium_link_probe)

    # VM-061 gate round 2, Finding 1 fix_instruction point 3: merge_yamlcpp.sh's
    # own post-merge audit only sees liboverlume.a itself -- this is the
    # other half, checking a real LINKED EXECUTABLE that (unlike the ROS node)
    # links a raw, un-merged cesium target directly for header access, so it
    # can independently catch the "un-renamed spdlog lands somewhere it can
    # collide" hazard the archive-only audit structurally cannot see.
    add_test(NAME cesium_link_probe_symbol_hygiene
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cesium_link_probe_symbols.sh
                     "${CMAKE_NM}" "$<TARGET_FILE:cesium_link_probe>")
endif()

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
    file(GLOB _overlume_test_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    list(REMOVE_ITEM _overlume_test_srcs "${_golden_cpp}")
    # VM-062: test_environment_stream.cpp calls the overlume::testing streaming
    # hooks (install_fixture_streaming_source, ecef_to_map_probe, ...),
    # DEFINED only in environment_stream.cpp -- which itself is excluded
    # from this build entirely when OVERLUME_ENABLE_CESIUM is OFF (see that
    # option's own comment above). Without this exclusion, the ordinary
    # build/ tree (option OFF) would try to link a test binary
    # against symbols that don't exist anywhere in it -- exactly the
    # "invisible in build/" hard rule this task is bound by.
    if(NOT OVERLUME_ENABLE_CESIUM)
        list(REMOVE_ITEM _overlume_test_srcs
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_environment_stream.cpp")
    endif()
    foreach(_test_src ${_overlume_test_srcs})
        get_filename_component(_test_name "${_test_src}" NAME_WE)
        set(_test_extra_srcs "")
        if(EXISTS "${_golden_cpp}")
            set(_test_extra_srcs "${_golden_cpp}")
        endif()
        add_executable(${_test_name} "${_test_src}" ${_test_extra_srcs})
        # -I src: lets test TUs include internal-only headers like
        # scene_buffer.hpp (Task 1/VM-010). Deliberately NOT the Filament
        # include dir (that stays overlume's PRIVATE-only) — internal
        # headers that pull in <filament/...> (e.g. renderer_internal.hpp)
        # are still off-limits to tests for that reason; see that header's
        # own comment.
        target_include_directories(${_test_name} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src" "${STB_DIR}")
        target_link_libraries(${_test_name} PRIVATE overlume gtest gtest_main EGL)
        # OVERLUME_TEST_DATA_DIR: resolvable-path prefix for golden/fixture
        # literals (gtest_discover_tests' WORKING_DIRECTORY is the build
        # dir, not this source dir). OVERLUME_DEFAULT_THEME_DIR: kThemeDir's
        # backing macro (tests/test_paths.hpp) — Task 2 Step 5a.
        target_compile_definitions(${_test_name} PRIVATE
            OVERLUME_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
            OVERLUME_DEFAULT_THEME_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")
        include(GoogleTest)
        gtest_discover_tests(${_test_name} DISCOVERY_TIMEOUT 30)
    endforeach()
endif()

# tools/ — standalone binaries linking overlume, one executable per .cpp
# (VM-041, Epic 5: viz_benchmark.cpp prints render_ms
# p50/p99 per quality preset for VM-040's governor to tune against later).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    file(GLOB _overlume_tool_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/*.cpp")
    foreach(_tool_src ${_overlume_tool_srcs})
        get_filename_component(_tool_name "${_tool_src}" NAME_WE)
        add_executable(${_tool_name} "${_tool_src}")
        target_link_libraries(${_tool_name} PRIVATE overlume)
    endforeach()
endif()

# Task 4 (open-source restructure plan): repo-root examples/ -- C++
# programs against overlume's PUBLIC API only (<overlume/api.h>,
# <overlume/scene.h>), one directory up from this project's own source dir
# (this project's root IS overlume/, so ${PROJECT_SOURCE_DIR}/../examples
# is the repo root's examples/). The EXISTS guard keeps a library-only
# checkout (vendored subtree, no repo-root examples/) configurable.
option(OVERLUME_BUILD_EXAMPLES "Build the API examples" ON)
if(OVERLUME_BUILD_EXAMPLES AND EXISTS "${PROJECT_SOURCE_DIR}/../examples/CMakeLists.txt")
    add_subdirectory(${PROJECT_SOURCE_DIR}/../examples ${CMAKE_BINARY_DIR}/examples)
endif()
