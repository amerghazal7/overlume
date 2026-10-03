# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

cmake_minimum_required(VERSION 3.18)

project(overlume VERSION 0.1.0 LANGUAGES CXX)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/overlume/version.h"
     _overlume_version_h_defines
     REGEX "^#define OVERLUME_VERSION_(MAJOR|MINOR|PATCH) [0-9]+$")
foreach(_line ${_overlume_version_h_defines})
    if(_line MATCHES "^#define OVERLUME_VERSION_MAJOR ([0-9]+)$")
        set(_overlume_version_h_major "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^#define OVERLUME_VERSION_MINOR ([0-9]+)$")
        set(_overlume_version_h_minor "${CMAKE_MATCH_1}")
    elseif(_line MATCHES "^#define OVERLUME_VERSION_PATCH ([0-9]+)$")
        set(_overlume_version_h_patch "${CMAKE_MATCH_1}")
    endif()
endforeach()
if(NOT DEFINED _overlume_version_h_major OR NOT DEFINED _overlume_version_h_minor
   OR NOT DEFINED _overlume_version_h_patch)
    message(FATAL_ERROR
        "overlume: could not parse OVERLUME_VERSION_{MAJOR,MINOR,PATCH} out "
        "of include/overlume/version.h")
endif()
if(NOT _overlume_version_h_major EQUAL PROJECT_VERSION_MAJOR OR
   NOT _overlume_version_h_minor EQUAL PROJECT_VERSION_MINOR OR
   NOT _overlume_version_h_patch EQUAL PROJECT_VERSION_PATCH)
    message(FATAL_ERROR
        "overlume: version drift -- project(overlume VERSION "
        "${PROJECT_VERSION}) vs include/overlume/version.h's "
        "${_overlume_version_h_major}.${_overlume_version_h_minor}."
        "${_overlume_version_h_patch}. Keep both in lockstep.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Linux only: the release/dev toolchain is clang + static libc++ (Apple, Windows
# and the Android NDK bring their own C++ runtime and compiler).
set(_overlume_cxx_runtime_libs "")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
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

    # Cross builds name the target triple so the probe looks at the target's runtime.
    set(_libcxx_target_arg "")
    if(CMAKE_CXX_COMPILER_TARGET)
        set(_libcxx_target_arg "--target=${CMAKE_CXX_COMPILER_TARGET}")
    endif()
    execute_process(
        COMMAND "${CMAKE_CXX_COMPILER}" ${_libcxx_target_arg} -stdlib=libc++ -print-file-name=libc++.a
        OUTPUT_VARIABLE _libcxx_a
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT EXISTS "${_libcxx_a}")
        # LLVM release tarball layout: <root>/lib/<triple>/libc++.a, which clang
        # does not report via -print-file-name.
        execute_process(
            COMMAND "${CMAKE_CXX_COMPILER}" ${_libcxx_target_arg} -print-target-triple
            OUTPUT_VARIABLE _libcxx_triple
            OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        get_filename_component(_libcxx_cxx_bin "${CMAKE_CXX_COMPILER}" REALPATH)
        get_filename_component(_libcxx_cxx_bin "${_libcxx_cxx_bin}" DIRECTORY)
        foreach(_libcxx_triple_name "${_libcxx_triple}" "${CMAKE_SYSTEM_PROCESSOR}-unknown-linux-gnu")
            if(EXISTS "${_libcxx_cxx_bin}/../lib/${_libcxx_triple_name}/libc++.a")
                set(_libcxx_a "${_libcxx_cxx_bin}/../lib/${_libcxx_triple_name}/libc++.a")
                break()
            endif()
        endforeach()
    endif()
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
    set(_overlume_cxx_runtime_libs "${_libcxx_a}" "${_libcxxabi_a}" "${_libunwind_a}")
endif()

include(cmake/GetFilament.cmake)

option(OVERLUME_ENABLE_CESIUM "Build the cesium-native path (Epic 6 / VM-061). Requires the (now \
primary) clang-18 toolchain (scripts/setup_toolchain_cesium.sh). OFF gives a \
library-only build that never touches cesium-native (hosted CI uses OFF)." ON)
if(OVERLUME_ENABLE_CESIUM)
    include(cmake/GetCesiumNative.cmake)
endif()

include(FetchContent)
FetchContent_Declare(
    yamlcpp
    URL "https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz"
    URL_HASH SHA256=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yamlcpp)

set(STB_IMAGE_WRITE_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
set(STB_IMAGE_WRITE_SHA256 "cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05")
set(STB_IMAGE_COMMIT "${STB_IMAGE_WRITE_COMMIT}")
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

set(_matc "${FILAMENT_HOST_MATC}")
# Mobile (GLES) materials on Android, desktop GL elsewhere.
set(_matc_profile desktop)
if(ANDROID)
    set(_matc_profile mobile)
endif()
set(_generated_material_dir "${CMAKE_BINARY_DIR}/generated/overlume_materials")
file(MAKE_DIRECTORY "${_generated_material_dir}")
file(GLOB _material_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/assets/materials/*.mat")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_material_srcs})
foreach(_mat_src ${_material_srcs})
    get_filename_component(_mat_name "${_mat_src}" NAME_WE)
    set(_mat_out "${_generated_material_dir}/${_mat_name}.filamat")
    set(_mat_header "${_generated_material_dir}/${_mat_name}_filamat.h")
    if("${_mat_src}" IS_NEWER_THAN "${_mat_header}")
        execute_process(
            COMMAND "${_matc}" -a opengl -p ${_matc_profile} -o "${_mat_out}" "${_mat_src}"
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

file(GLOB_RECURSE OVERLUME_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# Headless back end, selected at compile time (EGL; Metal/WGL arrive with their ports).
if(APPLE)
    set(OVERLUME_PLATFORM metal)
elseif(WIN32)
    set(OVERLUME_PLATFORM wgl)
else()
    set(OVERLUME_PLATFORM egl)
endif()
set(OVERLUME_PLATFORM_LIBS "")
if(ANDROID)
    set(OVERLUME_PLATFORM_LIBS EGL GLESv3 log android)
elseif(OVERLUME_PLATFORM STREQUAL "egl")
    set(OVERLUME_PLATFORM_LIBS EGL)
endif()
list(FILTER OVERLUME_SOURCES EXCLUDE REGEX "/platform_[a-z]+\\.cpp$")
list(APPEND OVERLUME_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/platform_${OVERLUME_PLATFORM}.cpp")
if(ANDROID)
    # Filament's Android Release build is -fno-rtti, so libbackend.a carries no typeinfo for
    # OpenGLPlatform; the subclass in platform_egl.cpp must not ask for it.
    set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/src/platform_egl.cpp"
        PROPERTIES COMPILE_OPTIONS "-fno-rtti")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    list(REMOVE_ITEM OVERLUME_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
endif()

# One OBJECT library feeds both link products: `overlume` (static, merged
# archive; what the ROS node, tests and the gate link) and `overlume_shared`.
add_library(overlume_obj OBJECT ${OVERLUME_SOURCES})

if(OVERLUME_ENABLE_CESIUM)
    set(_overlume_vendor_merge_direct_targets
        Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync CesiumCurl)
endif()

set(_overlume_overlume_stream_objects "")
if(OVERLUME_ENABLE_CESIUM AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    add_library(overlume_stream OBJECT src/environment_stream.cpp)
    target_compile_features(overlume_stream PRIVATE cxx_std_20)
    target_link_libraries(overlume_stream PRIVATE
        Cesium3DTilesSelection CesiumGltfWriter CesiumGltfReader CesiumAsync
        CesiumCurl
        Filament::filament
    )
    target_include_directories(overlume_stream PRIVATE src include)
    set(_overlume_overlume_stream_objects $<TARGET_OBJECTS:overlume_stream>)
endif()

add_library(overlume STATIC $<TARGET_OBJECTS:overlume_obj> ${_overlume_overlume_stream_objects})
set_target_properties(overlume PROPERTIES LINKER_LANGUAGE CXX)
add_library(overlume::overlume ALIAS overlume)

if(OVERLUME_ENABLE_CESIUM)
    target_compile_options(overlume_obj PRIVATE -std=gnu++${CMAKE_CXX_STANDARD})
    target_compile_definitions(overlume_obj PRIVATE OVERLUME_ENABLE_CESIUM=1)
endif()

# Usage requirements only (an OBJECT library never links).
target_link_libraries(overlume_obj PRIVATE
    Filament::filament yaml-cpp::yaml-cpp ${_overlume_vendor_merge_direct_targets})
target_include_directories(overlume_obj PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_BINARY_DIR}/generated/overlume_materials")
target_compile_definitions(overlume_obj PRIVATE
    DEFAULT_THEME_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets/themes")

# Link line shared by both products (today's static link line, same order).
set(_overlume_private_link
    Filament::filament
    yaml-cpp::yaml-cpp
    ${_overlume_cxx_runtime_libs}
)
if(OVERLUME_ENABLE_CESIUM)
    if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
        set(_overlume_private_link
            "-Wl,--start-group" ${_overlume_private_link}
            ${_overlume_vendor_merge_direct_targets} "-Wl,--end-group")
    else()
        list(APPEND _overlume_private_link ${_overlume_vendor_merge_direct_targets})
    endif()
endif()

target_include_directories(overlume PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(overlume PRIVATE ${_overlume_private_link})

if(OVERLUME_ENABLE_CESIUM AND CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
    target_link_options(overlume INTERFACE "-Wl,--allow-multiple-definition")
endif()

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
    _overlume_collect_vendor_merge_archives(_overlume_vendor_merge_archives
        ${_overlume_vendor_merge_direct_targets})
    list(REMOVE_DUPLICATES _overlume_vendor_merge_archives)

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

# Apple/Windows: the static product ships without the symbol-rename step
# (the static config warns consumers); see merge_yamlcpp.sh for the tools it needs.
if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
    add_custom_command(TARGET overlume POST_BUILD
        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/merge_yamlcpp.sh"
                "${CMAKE_AR}" "${CMAKE_LINKER}" "${CMAKE_OBJCOPY}" "${CMAKE_NM}"
                "${CMAKE_BINARY_DIR}/_yamlmerge"
                "${CMAKE_BINARY_DIR}/overlume_merged.o"
                "$<TARGET_FILE:overlume>"
                $<TARGET_OBJECTS:overlume_obj>
                ${_overlume_overlume_stream_objects}
                --
                "$<TARGET_FILE:yaml-cpp>"
                ${_overlume_vendor_merge_archives}
        COMMAND_EXPAND_LISTS VERBATIM
        COMMENT "overlume: merging yaml-cpp + cesium-native + spdlog/fmt, renaming their vendored symbols (Epic 2 Task 1 Step 0.2; extended Epic 6 VM-061 Step 5)")
endif()

option(OVERLUME_BUILD_SHARED "Build the self-contained shared liboverlume (deps baked in, only overlume::* exported)" ON)
if(OVERLUME_BUILD_SHARED)
    add_library(overlume_shared SHARED $<TARGET_OBJECTS:overlume_obj> ${_overlume_overlume_stream_objects})
    set_target_properties(overlume_shared PROPERTIES
        OUTPUT_NAME overlume EXPORT_NAME overlume LINKER_LANGUAGE CXX)
    # Android packages plain liboverlume.so (APKs and Prefab cannot carry versioned sonames).
    if(NOT ANDROID)
        set_target_properties(overlume_shared PROPERTIES
            VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})
    endif()
    target_include_directories(overlume_shared PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
    # libc++ last (CMake emits an INTERFACE target's libs after the direct
    # items, in order): it must resolve what Filament/Cesium leave undefined.
    add_library(overlume_cxx_runtime INTERFACE)
    target_link_libraries(overlume_cxx_runtime INTERFACE
        ${_overlume_cxx_runtime_libs})
    set(_overlume_shared_link Filament::filament yaml-cpp::yaml-cpp)
    if(OVERLUME_ENABLE_CESIUM)
        if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
            set(_overlume_shared_link "-Wl,--start-group" ${_overlume_shared_link}
                ${_overlume_vendor_merge_direct_targets} "-Wl,--end-group")
        else()
            list(APPEND _overlume_shared_link ${_overlume_vendor_merge_direct_targets})
        endif()
    endif()
    target_link_libraries(overlume_shared PRIVATE ${_overlume_shared_link} overlume_cxx_runtime)
    if(OVERLUME_ENABLE_CESIUM AND CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
        target_link_options(overlume_shared PRIVATE "-Wl,--allow-multiple-definition")
    endif()
    if(CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
        target_link_options(overlume_shared PRIVATE
            "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/cmake/overlume_exports.map"
            "-Wl,--exclude-libs,ALL" "-Wl,--no-undefined")
        set_property(TARGET overlume_shared APPEND PROPERTY LINK_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/overlume_exports.map")
    endif()
endif()

enable_testing()

add_test(NAME check_pod_header
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_pod_header.sh)
set_tests_properties(check_pod_header PROPERTIES LABELS "cpu")

add_test(NAME static_link_flags
         COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/test_static_link_flags.cmake)
set_tests_properties(static_link_flags PROPERTIES LABELS "cpu")

if(OVERLUME_BUILD_SHARED AND CMAKE_SYSTEM_NAME MATCHES "Linux|Android")
    add_test(NAME shared_exports
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_shared_exports.sh
                     "${CMAKE_NM}" "${CMAKE_READELF}" "$<TARGET_FILE:overlume_shared>")
    set_tests_properties(shared_exports PROPERTIES LABELS "cpu")
endif()

add_test(NAME cesium_triplet_mapping
         COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cesium_triplets.cmake)
set_tests_properties(cesium_triplet_mapping PROPERTIES LABELS "cpu")

add_executable(filament_link_probe smoke/filament_link_probe.cpp)
target_include_directories(filament_link_probe PRIVATE "${FILAMENT_ROOT}/include")
target_link_libraries(filament_link_probe PRIVATE overlume)
add_test(NAME filament_link_probe COMMAND filament_link_probe)
set_tests_properties(filament_link_probe PROPERTIES LABELS "cpu")

if(OVERLUME_ENABLE_CESIUM)
    add_executable(cesium_link_probe smoke/cesium_link_probe.cpp)
    set_target_properties(cesium_link_probe PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES)
    target_link_libraries(cesium_link_probe PRIVATE
        overlume CesiumGeospatial CesiumGltfReader)
    add_test(NAME cesium_link_probe COMMAND cesium_link_probe)
    set_tests_properties(cesium_link_probe PROPERTIES LABELS "cpu")

    add_test(NAME cesium_link_probe_symbol_hygiene
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_cesium_link_probe_symbols.sh
                     "${CMAKE_NM}" "$<TARGET_FILE:cesium_link_probe>")
    set_tests_properties(cesium_link_probe_symbol_hygiene PROPERTIES LABELS "cpu")
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
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

    set(_golden_cpp "${CMAKE_CURRENT_SOURCE_DIR}/tests/golden.cpp")
    file(GLOB _overlume_test_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
    list(REMOVE_ITEM _overlume_test_srcs "${_golden_cpp}")
    if(NOT OVERLUME_ENABLE_CESIUM)
        list(REMOVE_ITEM _overlume_test_srcs
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_environment_stream.cpp")
    endif()
    # Where tests find fixtures at run time (Android sets the on-device path).
    set(OVERLUME_TEST_DATA_DIR_RUNTIME "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING
        "Run-time path of the overlume source tree's test data")
    # Where tests write their *_actual.png / scratch files (Android has no /tmp).
    set(OVERLUME_TEST_TMP_DIR_RUNTIME "/tmp" CACHE STRING "Run-time scratch directory for tests")
    foreach(_test_src ${_overlume_test_srcs})
        get_filename_component(_test_name "${_test_src}" NAME_WE)
        set(_test_extra_srcs "")
        if(EXISTS "${_golden_cpp}")
            set(_test_extra_srcs "${_golden_cpp}")
        endif()
        add_executable(${_test_name} "${_test_src}" ${_test_extra_srcs})
        target_include_directories(${_test_name} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src" "${STB_DIR}")
        target_link_libraries(${_test_name} PRIVATE overlume gtest gtest_main ${OVERLUME_PLATFORM_LIBS})
        if(_test_name STREQUAL "test_gltf_normals")
            target_include_directories(${_test_name} PRIVATE
                $<TARGET_PROPERTY:yaml-cpp::yaml-cpp,INTERFACE_INCLUDE_DIRECTORIES>)
        endif()
        target_compile_definitions(${_test_name} PRIVATE
            OVERLUME_TEST_DATA_DIR="${OVERLUME_TEST_DATA_DIR_RUNTIME}"
            OVERLUME_TMP_DIR="${OVERLUME_TEST_TMP_DIR_RUNTIME}"
            $<$<BOOL:${ANDROID}>:OVERLUME_GOLDEN_SSIM_MIN=0.97>
            OVERLUME_DEFAULT_THEME_DIR="${OVERLUME_TEST_DATA_DIR_RUNTIME}/assets/themes")
        include(GoogleTest)
        # Cross builds cannot run the binary at build time: list its tests when ctest loads.
        if(CMAKE_CROSSCOMPILING)
            set(_gtest_discovery DISCOVERY_MODE PRE_TEST)
        else()
            set(_gtest_discovery "")
        endif()
        if(_test_name STREQUAL "test_renderer_projection" OR
           _test_name STREQUAL "test_renderer_quality_presets" OR
           _test_name STREQUAL "test_readback_orientation")
            gtest_discover_tests(${_test_name} ${_gtest_discovery} DISCOVERY_TIMEOUT 30 PROPERTIES LABELS "gpu")
        else()
            gtest_discover_tests(${_test_name} ${_gtest_discovery} DISCOVERY_TIMEOUT 30 PROPERTIES LABELS "cpu")
        endif()
    endforeach()
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    file(GLOB _overlume_tool_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tools/*.cpp")
    foreach(_tool_src ${_overlume_tool_srcs})
        get_filename_component(_tool_name "${_tool_src}" NAME_WE)
        add_executable(${_tool_name} "${_tool_src}")
        target_link_libraries(${_tool_name} PRIVATE overlume ${OVERLUME_PLATFORM_LIBS})
    endforeach()
endif()

option(OVERLUME_BUILD_EXAMPLES "Build the API examples" ON)
if(OVERLUME_BUILD_EXAMPLES AND EXISTS "${PROJECT_SOURCE_DIR}/../examples/CMakeLists.txt")
    add_subdirectory(${PROJECT_SOURCE_DIR}/../examples ${CMAKE_BINARY_DIR}/examples)
endif()

find_program(DOXYGEN_EXECUTABLE doxygen HINTS "$ENV{HOME}/.local/bin")
if(DOXYGEN_EXECUTABLE)
    option(OVERLUME_BUILD_DOCS "Build the docs target (Doxygen API reference)" ON)
else()
    option(OVERLUME_BUILD_DOCS "Build the docs target (Doxygen API reference)" OFF)
    message(STATUS
        "overlume: no doxygen executable found (checked PATH and "
        "~/.local/bin) -- OVERLUME_BUILD_DOCS defaults OFF. Install doxygen "
        "or pass -DDOXYGEN_EXECUTABLE=/path/to/doxygen to enable it.")
endif()

if(OVERLUME_BUILD_DOCS AND EXISTS "${PROJECT_SOURCE_DIR}/../Doxyfile.in")
    if(NOT DOXYGEN_EXECUTABLE)
        message(FATAL_ERROR
            "overlume: OVERLUME_BUILD_DOCS=ON but no doxygen executable was "
            "found. Install doxygen, pass -DDOXYGEN_EXECUTABLE=/path/to/doxygen, "
            "or configure with -DOVERLUME_BUILD_DOCS=OFF.")
    endif()
    include(cmake/GetDoxygenAwesome.cmake)
    get_filename_component(OVERLUME_REPO_ROOT "${PROJECT_SOURCE_DIR}/.." ABSOLUTE)
    configure_file("${OVERLUME_REPO_ROOT}/Doxyfile.in" "${CMAKE_BINARY_DIR}/Doxyfile" @ONLY)
    add_custom_target(docs
        COMMAND "${DOXYGEN_EXECUTABLE}" "${CMAKE_BINARY_DIR}/Doxyfile"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        COMMENT "overlume: generating API docs (Doxygen) into ${CMAKE_BINARY_DIR}/docs/html"
        VERBATIM)
elseif(OVERLUME_BUILD_DOCS)
    message(STATUS "overlume: Doxyfile.in not found next to the library (library-only checkout) -- docs target skipped")
endif()

include(cmake/OverlumeInstall.cmake)

include(cmake/OverlumePackaging.cmake)
