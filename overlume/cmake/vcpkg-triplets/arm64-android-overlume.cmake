# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# arm64-android-overlume: static libs, built by the matching release job (see docs/plans/2026-10-02-cross-platform-release.md).

set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_CMAKE_SYSTEM_NAME Android)
set(VCPKG_CMAKE_SYSTEM_VERSION 26)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../vcpkg-android-toolchain.cmake")
set(VCPKG_C_FLAGS -g0)  # the NDK adds -g; the static package would carry it
set(VCPKG_CXX_FLAGS -g0)
