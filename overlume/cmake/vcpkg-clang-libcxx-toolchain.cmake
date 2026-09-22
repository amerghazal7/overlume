# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set(CMAKE_SYSTEM_NAME "Linux" CACHE STRING "")
set(CMAKE_SYSTEM_PROCESSOR "x86_64" CACHE STRING "")

include("${CMAKE_CURRENT_LIST_DIR}/clang18-toolchain-common.cmake")
set(_overlume_vcpkg_libdir1 "${_overlume_clang18_libdir1}")
set(_overlume_vcpkg_libdir2 "${_overlume_clang18_libdir2}")

set(CMAKE_C_COMPILER "${_overlume_clang18_wrap_clang}" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_overlume_clang18_wrap_clangxx}" CACHE FILEPATH "" FORCE)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++ -fPIC")
set(CMAKE_C_FLAGS_INIT "-fPIC")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-stdlib=libc++ -Wl,--disable-new-dtags -Wl,-rpath,${_overlume_vcpkg_libdir1} -Wl,-rpath,${_overlume_vcpkg_libdir2}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT
    "-stdlib=libc++ -Wl,--disable-new-dtags -Wl,-rpath,${_overlume_vcpkg_libdir1} -Wl,-rpath,${_overlume_vcpkg_libdir2}")
