# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# aarch64 cross toolchain, run on x86_64: the x86_64 LLVM tarball at /opt/llvm
# (clang + lld) targeting an AlmaLinux 8 aarch64 sysroot (glibc 2.28), with the
# aarch64 libc++ built into /opt/llvm/lib/aarch64-unknown-linux-gnu. Everything
# but the target lives in toolchain-llvm-release.cmake. See
# tools/release/linux/Dockerfile.cross-aarch64 for how the image is made.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(OVERLUME_RELEASE_TRIPLE aarch64-linux-gnu)
if(DEFINED ENV{OVERLUME_SYSROOT_AARCH64} AND NOT "$ENV{OVERLUME_SYSROOT_AARCH64}" STREQUAL "")
    set(OVERLUME_RELEASE_SYSROOT "$ENV{OVERLUME_SYSROOT_AARCH64}")
else()
    set(OVERLUME_RELEASE_SYSROOT "/opt/sysroot-aarch64")
endif()
include("${CMAKE_CURRENT_LIST_DIR}/toolchain-llvm-release.cmake")
