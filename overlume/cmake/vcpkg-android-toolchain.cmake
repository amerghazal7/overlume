# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# vcpkg chainload for the *-android-overlume triplets. vcpkg's own scripts/toolchains/android.cmake
# never sets ANDROID_ABI, so every port (and configure-make ports like openssl, which read the
# detected CMAKE_ANDROID_ARCH) would be built for the NDK default, armeabi-v7a. Pin the ABI from
# the triplet's architecture, then defer to vcpkg's toolchain.
if(VCPKG_TARGET_ARCHITECTURE STREQUAL "arm64")
    set(ANDROID_ABI arm64-v8a CACHE STRING "")
elseif(VCPKG_TARGET_ARCHITECTURE STREQUAL "arm")
    set(ANDROID_ABI armeabi-v7a CACHE STRING "")
elseif(VCPKG_TARGET_ARCHITECTURE STREQUAL "x64")
    set(ANDROID_ABI x86_64 CACHE STRING "")
elseif(VCPKG_TARGET_ARCHITECTURE STREQUAL "x86")
    set(ANDROID_ABI x86 CACHE STRING "")
else()
    message(FATAL_ERROR "overlume: no Android ABI for VCPKG_TARGET_ARCHITECTURE '${VCPKG_TARGET_ARCHITECTURE}'")
endif()
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ANDROID_ABI)
# The includer is <vcpkg root>/scripts/buildsystems/vcpkg.cmake.
get_filename_component(_ov_vcpkg_root "${CMAKE_PARENT_LIST_FILE}/../../.." ABSOLUTE)
include("${_ov_vcpkg_root}/scripts/toolchains/android.cmake")
