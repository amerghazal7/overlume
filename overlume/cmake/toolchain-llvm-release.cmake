# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Release toolchain: the official LLVM release tarball (default /opt/llvm, the
# layout of tools/release/linux/Dockerfile) with libc++, same flags as
# toolchain-clang-libcxx.cmake plus a statically linked libgcc.

if(DEFINED ENV{OVERLUME_LLVM_ROOT} AND NOT "$ENV{OVERLUME_LLVM_ROOT}" STREQUAL "")
    set(_overlume_llvm_root "$ENV{OVERLUME_LLVM_ROOT}")
else()
    set(_overlume_llvm_root "/opt/llvm")
endif()
if(NOT EXISTS "${_overlume_llvm_root}/bin/clang++")
    message(FATAL_ERROR "overlume: no LLVM at ${_overlume_llvm_root} "
        "(set OVERLUME_LLVM_ROOT; the release container provides /opt/llvm).")
endif()
# Visible to vcpkg's chainload toolchain (clang18-toolchain-common.cmake).
set(ENV{OVERLUME_LLVM_ROOT} "${_overlume_llvm_root}")

set(CMAKE_C_COMPILER "${_overlume_llvm_root}/bin/clang" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_overlume_llvm_root}/bin/clang++" CACHE FILEPATH "" FORCE)

# The release container builds Filament from source (no portable prebuilt).
set(OVERLUME_FILAMENT_FROM_SOURCE ON CACHE BOOL "" FORCE)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++ -static-libgcc -fuse-ld=lld")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++ -static-libgcc -fuse-ld=lld")

# lld (in the tarball) applies the extern "C++" export map consistently; GNU ld
# 2.30 on Alma 8 leaks weak libc++ instantiations when nothing inlines them.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
