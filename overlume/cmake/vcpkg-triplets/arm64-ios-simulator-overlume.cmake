# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# arm64-ios-simulator-overlume: static libs, built by the matching release job (see docs/plans/2026-10-02-cross-platform-release.md).

set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_CMAKE_SYSTEM_NAME iOS)
set(VCPKG_OSX_SYSROOT iphonesimulator)
set(VCPKG_OSX_ARCHITECTURES arm64)
set(VCPKG_OSX_DEPLOYMENT_TARGET 15.0)
