# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set(CESIUM_NATIVE_VERSION "0.64.0")
set(CESIUM_NATIVE_URL
    "https://github.com/CesiumGS/cesium-native/archive/refs/tags/v${CESIUM_NATIVE_VERSION}.tar.gz")
set(CESIUM_NATIVE_SHA256
    "f3629345db4cb7412380cc31dea502aeb9e2eca75129ccbc04b7628970031c11")

set(VCPKG_OVERLAY_TRIPLETS "${CMAKE_CURRENT_LIST_DIR}/vcpkg-triplets" CACHE STRING "" FORCE)
include("${CMAKE_CURRENT_LIST_DIR}/CesiumTriplet.cmake")
set(_overlume_cn_arch "${CMAKE_SYSTEM_PROCESSOR}")
if(ANDROID)
    set(_overlume_cn_arch "${CMAKE_ANDROID_ARCH_ABI}")
elseif(APPLE AND CMAKE_OSX_ARCHITECTURES)
    list(GET CMAKE_OSX_ARCHITECTURES 0 _overlume_cn_arch)
endif()
overlume_cesium_triplet(_overlume_cn_triplet "${CMAKE_SYSTEM_NAME}" "${_overlume_cn_arch}" "${CMAKE_OSX_SYSROOT}")
set(VCPKG_TARGET_TRIPLET "${_overlume_cn_triplet}" CACHE STRING "" FORCE)
if(CMAKE_CROSSCOMPILING)
    # Helper tools run on the build host, not the target.
    overlume_cesium_triplet(_overlume_cn_host_triplet "${CMAKE_HOST_SYSTEM_NAME}" "${CMAKE_HOST_SYSTEM_PROCESSOR}" "")
    set(VCPKG_HOST_TRIPLET "${_overlume_cn_host_triplet}" CACHE STRING "" FORCE)
    set(ENV{VCPKG_DEFAULT_HOST_TRIPLET} "${_overlume_cn_host_triplet}")
else()
    set(VCPKG_HOST_TRIPLET "${_overlume_cn_triplet}" CACHE STRING "" FORCE)
endif()

set(CESIUM_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(CESIUM_COVERAGE_ENABLED OFF CACHE BOOL "" FORCE)
set(CESIUM_ENABLE_CLANG_TIDY OFF CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_STATIC_LIBS OFF CACHE BOOL "" FORCE)
set(CESIUM_INSTALL_HEADERS OFF CACHE BOOL "" FORCE)

set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(GLOB _overlume_cn_libstdcxx_dev_dirs "/usr/lib/gcc/*/*")
    set(_overlume_cn_libstdcxx_dev_dir "")
    foreach(_d ${_overlume_cn_libstdcxx_dev_dirs})
        if(EXISTS "${_d}/libstdc++.so" AND IS_DIRECTORY "${_d}")
            set(_overlume_cn_libstdcxx_dev_dir "${_d}")
            break()
        endif()
    endforeach()
    if(_overlume_cn_libstdcxx_dev_dir)
        set(ENV{LIBRARY_PATH} "${_overlume_cn_libstdcxx_dev_dir}:$ENV{LIBRARY_PATH}")
    else()
        message(WARNING
            "overlume/GetCesiumNative: no /usr/lib/gcc/*/*/libstdc++.so "
            "dev symlink found anywhere -- vcpkg's own scripts/detect_compiler "
            "pseudo-port (see comment above) may fail to link its bare "
            "no-flags compiler probe on this box.")
    endif()

    set(ENV{LD_LIBRARY_PATH} "${_libcxx_lib_dir}:$ENV{LD_LIBRARY_PATH}")
endif()

if(ANDROID AND CMAKE_ANDROID_ARCH_ABI STREQUAL "x86")
    # KTX-Software refuses 32-bit x86 at configure time; this overlay port patches that guard out.
    set(VCPKG_OVERLAY_PORTS "${CMAKE_CURRENT_LIST_DIR}/vcpkg-ports" CACHE STRING "" FORCE)
endif()

if(ANDROID)
    # openssl's Configure takes its compiler from $CC (its Android config is patched to skip the NDK
    # lookup); the target triple and sysroot arrive through vcpkg's detected flags.
    set(ENV{CC} "${CMAKE_C_COMPILER}")
endif()

set(CMAKE_FIND_PACKAGE_TARGETS_GLOBAL TRUE)
include(FetchContent)
FetchContent_Declare(
    cesium-native
    URL "${CESIUM_NATIVE_URL}"
    URL_HASH SHA256=${CESIUM_NATIVE_SHA256})
FetchContent_MakeAvailable(cesium-native)

function(_overlume_disable_warnings_as_errors_recursive _dir)
    get_property(_targets DIRECTORY "${_dir}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(_tgt ${_targets})
        set_target_properties(${_tgt} PROPERTIES COMPILE_WARNING_AS_ERROR OFF)
    endforeach()
    get_property(_subdirs DIRECTORY "${_dir}" PROPERTY SUBDIRECTORIES)
    foreach(_subdir ${_subdirs})
        _overlume_disable_warnings_as_errors_recursive("${_subdir}")
    endforeach()
endfunction()
_overlume_disable_warnings_as_errors_recursive("${cesium-native_SOURCE_DIR}")
