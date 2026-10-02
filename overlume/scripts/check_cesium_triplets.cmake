# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# cmake -P check: target -> cesium-native overlay triplet mapping, and that
# every mapped triplet has a file under cmake/vcpkg-triplets.
get_filename_component(_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
include("${_root}/cmake/CesiumTriplet.cmake")

set(_fail FALSE)
macro(_expect _sys _arch _sdk _want)
    overlume_cesium_triplet(_got "${_sys}" "${_arch}" "${_sdk}")
    if(NOT _got STREQUAL "${_want}")
        message(SEND_ERROR "FAIL ${_sys}/${_arch}/${_sdk}: got '${_got}', want '${_want}'")
        set(_fail TRUE)
    elseif(NOT EXISTS "${_root}/cmake/vcpkg-triplets/${_got}.cmake")
        message(SEND_ERROR "FAIL ${_got}: no triplet file")
        set(_fail TRUE)
    endif()
endmacro()

set(_sdk "")
_expect(Linux x86_64 "${_sdk}" x64-linux-clang-libcxx)
_expect(Linux aarch64 "${_sdk}" arm64-linux-clang-libcxx)
_expect(Android arm64-v8a "${_sdk}" arm64-android-overlume)
_expect(Android armeabi-v7a "${_sdk}" arm-android-overlume)
_expect(Android x86_64 "${_sdk}" x64-android-overlume)
_expect(Android x86 "${_sdk}" x86-android-overlume)
_expect(Darwin arm64 "${_sdk}" arm64-osx-overlume)
_expect(Darwin x86_64 "${_sdk}" x64-osx-overlume)
_expect(iOS arm64 "${_sdk}" arm64-ios-overlume)
set(_sdk iphonesimulator)
_expect(iOS arm64 "${_sdk}" arm64-ios-simulator-overlume)
_expect(iOS x86_64 "${_sdk}" x64-ios-simulator-overlume)
set(_sdk "")
_expect(Windows AMD64 "${_sdk}" x64-windows-overlume)
_expect(Windows ARM64 "${_sdk}" arm64-windows-overlume)

if(_fail)
    message(FATAL_ERROR "cesium triplet mapping check FAILED")
endif()
message(STATUS "cesium triplet mapping check PASS")
