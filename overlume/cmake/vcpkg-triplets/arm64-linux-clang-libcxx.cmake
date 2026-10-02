# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# aarch64 target, always cross-compiled from x86_64 (the release image's
# toolchain-llvm-release-aarch64.cmake + Alma 8 sysroot).

set(VCPKG_TARGET_ARCHITECTURE arm64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../vcpkg-llvm-release-aarch64-toolchain.cmake")
