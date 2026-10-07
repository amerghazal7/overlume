# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# arm64-windows-overlume: static libs, built by the matching release job (see docs/plans/2026-10-02-cross-platform-release.md).

set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CRT_LINKAGE dynamic)
# No VCPKG_CMAKE_SYSTEM_NAME: for plain Windows vcpkg uses its own MSVC toolchain (setting it selects the generic path, which finds no compiler).
