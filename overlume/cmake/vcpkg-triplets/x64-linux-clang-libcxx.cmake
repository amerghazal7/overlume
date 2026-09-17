# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# x64-linux-clang-libcxx.vcpkg triplet (VM-061) — every cesium-native vcpkg
# port on this platform is built by vcpkg with ITS OWN triplet, not with the
# enclosing project's CMAKE_CXX_FLAGS, so a stock x64-linux port set would
# be gcc/libstdc++ under this box's default vcpkg bootstrap toolchain (the
# yaml-cpp/GoogleTest precedent this whole file exists to avoid,
# CMakeLists.txt:77-93,240-261). This overlay triplet forces the same
# clang++ + -stdlib=libc++ the parent project enforces via
# VCPKG_CHAINLOAD_TOOLCHAIN_FILE, static linkage per Decision 2.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/../vcpkg-clang-libcxx-toolchain.cmake")

# VM-061 Step 6 (2026-09-15), finding: these static archives now end up
# merged (ld -r, scripts/merge_yamlcpp.sh) into liboverlume.a, which
# the ROS node links into overlume_ros's OWN SHARED library
# (overlume_node_lib) -- something no consumer of overlume did
# before Epic 6 (every prior consumer, including this library's own test
# suite, is an EXECUTABLE, never a .so). Without -fPIC, spdlog's
# thread_local `os.cpp` tid cache compiles with a TLS access model
# ("Local Exec"/TPOFF32 relocations, confirmed via readelf -r -- NOT
# "Initial Exec" as first assumed) that a shared object cannot carry --
# confirmed directly: "relocation R_X86_64_TPOFF32 against
# `overlume_vendored__ZGVZN6spdlog7details2os9thread_idEvE3tid' can not be used
# when making a shared object; recompile with -fPIC" building
# overlume_node_lib.so.
#
# TRIED AND REVERTED: VCPKG_C_FLAGS/VCPKG_CXX_FLAGS set HERE (this triplet
# file). Does NOTHING under this triplet -- those two variables are read and
# injected into CMAKE_CXX_FLAGS_INIT only by vcpkg's OWN STOCK toolchain
# script, and DEVIATION #2 in vcpkg-clang-libcxx-toolchain.cmake already
# documents that VCPKG_CHAINLOAD_TOOLCHAIN_FILE REPLACES that stock script
# ENTIRELY for this triplet, not layers on top of it -- so it never runs and
# never reads these two variables. Confirmed directly: even after this
# triplet file's own content changed (invalidating vcpkg's binary cache,
# forcing a real rebuild) and the stale package-tracking DB was cleared too,
# `vcpkg install spdlog --debug` showed zero `-fPIC` anywhere in the actual
# compile command lines. The fix belongs in the CHAINLOADED file instead
# (vcpkg-clang-libcxx-toolchain.cmake's own CMAKE_CXX_FLAGS_INIT/
# CMAKE_C_FLAGS_INIT -- that file's own DEVIATION #5 records this same
# story) -- it actually runs for every port build.
