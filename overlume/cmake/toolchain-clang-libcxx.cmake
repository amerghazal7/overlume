# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# toolchain-clang-libcxx.cmake — selects the clang-18/libc++ toolchain
# overlume must build with (spec §2: this lib is clang/libc++, the
# gcc ROS node stays gcc/libstdc++, and the two must never mix at an ABI
# boundary — see include/overlume/api.h).
#
# VM-061 Step 6 (USER DECISION a, 2026-09-15): PRIMARY toolchain migrated
# clang-14 -> clang-18. Reason: cesium-native's vcpkg dependency ada-url
# hard-requires `std::ranges::replace`, which libc++-14 (and -15) do not
# implement (verified directly; clang-16/17 aren't packaged on this box's
# Ubuntu release) — clang-18 (apt.llvm.org, rootless,
# scripts/setup_toolchain_cesium.sh) is the smallest packaged jump that has
# it, and was already proven end-to-end in the dedicated `build-cesium/` tree
# (Task 2 Steps 1-6). Option (b) from Task 2's gate-round blocker — restructure
# GetCesiumNative.cmake around a nested ExternalProject_Add with its own
# clang-18 toolchain, importing archives as IMPORTED targets, keeping this
# file at clang-14 — was REJECTED by the user as a real architectural
# rewrite for no runtime benefit. This file now resolves to the SAME
# clang-18 rootless prefix `build-cesium/` already used: libc++'s ABI
# (`std::__1::`) is stable 14->18 (this project's own merge-script ABI
# check + Step 6's full node+gtest rebuild are the empirical proof, not just
# the theory), and overlume's own compiled objects never cross the
# POD boundary as C++ types anyway — so migrating the compiler version
# changes nothing about overlume's own correctness.
#
# Usage (unchanged):
#   scripts/setup_toolchain_cesium.sh   # once, bootstraps the clang-18 prefix
#   cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .
#   cmake --build build
#   ctest --test-dir build
#
# Resolution order for CMAKE_CXX_COMPILER:
#   1. A clang++ already on PATH, but only if it actually has a co-located
#      libc++ static archive (`clang++ -stdlib=libc++ -print-file-name=libc++.a`
#      resolves to a real file) — so a box with a real (root-installed)
#      clang+libc++-dev setup just works without the rootless prefix. (Note:
#      this escape hatch does not itself check for a C++20/std::ranges-capable
#      clang; on this box nothing is on PATH, so it never fires — a future
#      box that DOES have a system clang++ with libc++ but an old version
#      would need OVERLUME_ENABLE_CESIUM=OFF, same constraint as before this
#      migration, just newly worth naming now that ON is the common case.)
#   2. clang18-toolchain-common.cmake's rootless clang-18 wrapper — the SAME
#      resolution cmake/vcpkg-clang-libcxx-toolchain.cmake and the node's own
#      cross-toolchain import (overlume_ros/CMakeLists.txt)
#      use, factored into one file rather than duplicated a second/third time
#      now that all three need clang-18 instead of just the vcpkg one.
#
# This also bakes in -stdlib=libc++ for compile and link so the documented
# configure line above needs no other flags.
#
# The retired clang-14 bootstrap's OLD prefix (overlume-toolchain/) is no
# longer resolved by this file. It is left in place (harmless, unused) rather
# than deleted — scripts/merge_yamlcpp.sh's own ABI check already proved
# clang-14- and clang-18-built libc++ objects merge safely, so nothing
# downstream needed the split kept once the primary toolchain itself could
# just move to clang-18.

find_program(_overlume_path_clangxx NAMES clang++)
set(_overlume_chosen_clangxx "")
if(_overlume_path_clangxx)
    execute_process(
        COMMAND "${_overlume_path_clangxx}" -stdlib=libc++ -print-file-name=libc++.a
        OUTPUT_VARIABLE _overlume_path_libcxx_a
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(_overlume_path_libcxx_a AND EXISTS "${_overlume_path_libcxx_a}")
        set(_overlume_chosen_clangxx "${_overlume_path_clangxx}")
    endif()
endif()

if(NOT _overlume_chosen_clangxx)
    include("${CMAKE_CURRENT_LIST_DIR}/clang18-toolchain-common.cmake")
    set(_overlume_chosen_clangxx "${_overlume_clang18_wrap_clangxx}")
endif()

set(CMAKE_CXX_COMPILER "${_overlume_chosen_clangxx}" CACHE FILEPATH
    "clang++ (libc++) for overlume" FORCE)

# CMAKE_EXE_LINKER_FLAGS_INIT below (-stdlib=libc++ -nostdlib++) is global —
# it applies to every target CMake configures in this build, including any
# plain-C dependency that enables LANGUAGES C (e.g. googletest's
# CMakeLists.txt). If CMAKE_C_COMPILER were left at its default (system
# gcc), CMake's C-compiler try_compile sanity check would hand those
# clang-only flags to gcc and fail immediately ("unrecognized command-line
# option") before any of our own targets are even configured — confirmed by
# hitting exactly that. Pointing the C compiler at this same toolchain's
# clang (sibling of clang++) sidesteps it entirely: nothing in this
# directory or its fetched C dependencies needs gcc.
get_filename_component(_overlume_toolchain_bindir "${_overlume_chosen_clangxx}" DIRECTORY)
set(CMAKE_C_COMPILER "${_overlume_toolchain_bindir}/clang" CACHE FILEPATH
    "clang (matching CMAKE_CXX_COMPILER) for overlume's C deps" FORCE)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
# -nostdlib++: overlume links libc++/libc++abi/libunwind as explicit
# static archives (see CMakeLists.txt) instead of the driver's default
# dynamic -lc++, so the result needs no LD_LIBRARY_PATH/rpath to run.
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
