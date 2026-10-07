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

# Cross: <triple>-clang makes the bare compiler target the triple too.
if(OVERLUME_RELEASE_TRIPLE)
    set(_overlume_cc_prefix "${OVERLUME_RELEASE_TRIPLE}-")
else()
    set(_overlume_cc_prefix "")
endif()
set(CMAKE_C_COMPILER "${_overlume_llvm_root}/bin/${_overlume_cc_prefix}clang" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_overlume_llvm_root}/bin/${_overlume_cc_prefix}clang++" CACHE FILEPATH "" FORCE)

# The release container builds Filament from source (no portable prebuilt).
set(OVERLUME_FILAMENT_FROM_SOURCE ON CACHE BOOL "" FORCE)

# Cross builds (toolchain-llvm-release-aarch64.cmake) set OVERLUME_RELEASE_TRIPLE
# and OVERLUME_RELEASE_SYSROOT first; everything else below is shared, so the
# flags are never forked. libc++/libc++abi/libunwind for the triple live in
# ${LLVM_ROOT}/lib/<triple>/ (tools/release/linux/Dockerfile.cross-aarch64).
set(_overlume_cross_flags "")
if(OVERLUME_RELEASE_TRIPLE)
    if(NOT IS_DIRECTORY "${OVERLUME_RELEASE_SYSROOT}/usr/include")
        message(FATAL_ERROR "overlume: no sysroot at ${OVERLUME_RELEASE_SYSROOT} "
            "(the cross release image provides /opt/sysroot-aarch64).")
    endif()
    file(GLOB _overlume_gcc_dirs "${OVERLUME_RELEASE_SYSROOT}/usr/lib/gcc/*/[0-9]*")
    list(GET _overlume_gcc_dirs 0 _overlume_gcc_dir)
    set(CMAKE_SYSROOT "${OVERLUME_RELEASE_SYSROOT}")
    set(CMAKE_C_COMPILER_TARGET "${OVERLUME_RELEASE_TRIPLE}")
    set(CMAKE_CXX_COMPILER_TARGET "${OVERLUME_RELEASE_TRIPLE}")
    set(CMAKE_ASM_COMPILER_TARGET "${OVERLUME_RELEASE_TRIPLE}")  # Filament's bluegl .S
    # crtbegin*.o / libgcc*.a come from the sysroot's gcc, not from the host's.
    set(_overlume_cross_flags " --gcc-install-dir=${_overlume_gcc_dir}")
    set(CMAKE_FIND_ROOT_PATH "${OVERLUME_RELEASE_SYSROOT}")
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    # Host-arch binutils cannot read the target's objects.
    foreach(_t STRIP:llvm-strip OBJDUMP:llvm-objdump OBJCOPY:llvm-objcopy)
        string(REPLACE ":" ";" _t "${_t}")
        list(GET _t 0 _var)
        list(GET _t 1 _tool)
        set(CMAKE_${_var} "${_overlume_llvm_root}/bin/${_tool}" CACHE FILEPATH "" FORCE)
    endforeach()
    # ctest runs on the target (arm64 runner): its binutils, found via PATH.
    set(CMAKE_NM "nm" CACHE FILEPATH "" FORCE)
    set(CMAKE_READELF "readelf" CACHE FILEPATH "" FORCE)
endif()

set(CMAKE_C_FLAGS_INIT "${_overlume_cross_flags}")
set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++${_overlume_cross_flags}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++ -static-libgcc -fuse-ld=lld${_overlume_cross_flags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++ -static-libgcc -fuse-ld=lld${_overlume_cross_flags}")

# lld (in the tarball) applies the extern "C++" export map consistently; GNU ld
# 2.30 on Alma 8 leaks weak libc++ instantiations when nothing inlines them.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
