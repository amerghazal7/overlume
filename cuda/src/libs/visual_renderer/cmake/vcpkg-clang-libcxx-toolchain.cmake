# vcpkg-clang-libcxx-toolchain.cmake (VM-061) — the CHAINLOADED toolchain
# file the x64-linux-clang-libcxx vcpkg overlay triplet points at
# (VCPKG_CHAINLOAD_TOOLCHAIN_FILE). vcpkg invokes this per-port, via its own
# `cmake -P`/toolchain machinery, NOT via this project's own
# `cmake --toolchain` line — so it needs the minimal form vcpkg's chainload
# wants (CMAKE_C_COMPILER/CMAKE_CXX_COMPILER + -stdlib=libc++ on
# CMAKE_CXX_FLAGS_INIT), not a full project configure.
#
# DEVIATION (recorded, root-caused empirically, VM-061 Step 1): this file
# does NOT use this project's primary clang-14 toolchain
# (cmake/toolchain-clang-libcxx.cmake / scripts/setup_toolchain.sh), even
# though Decision 2 originally assumed it would. Reason: cesium-native's
# vcpkg dependency "ada-url" is C++20-only and its url_search_params-inl.h
# calls `std::ranges::replace` -- a ranges <algorithm> overload libc++-14
# does not implement. Confirmed directly (both fail identically):
#   clang++-14 -stdlib=libc++ -std=c++20: "no member named 'replace' in
#     namespace 'std::ranges'"
#   clang++-15 -stdlib=libc++ -std=c++20 (also apt-installable here): same
#     error -- libc++-15 doesn't have it either.
#   clang++-18 (apt.llvm.org, no root needed): compiles AND runs correctly.
# clang-16/17 are not packaged in Ubuntu 22.04's own apt repos at all (no
# candidate). This is the plan's own named fallback for exactly this gap
# (Decision 15.2c: "a newer local clang via scripts/setup_toolchain.sh") --
# scripts/setup_toolchain_cesium.sh bootstraps clang-18 root-lessly from
# apt.llvm.org (read its header comment for the full reasoning, including
# why mixing clang-18-built cesium archives with this project's own
# clang-14-built objects in one merged archive is safe: libc++'s stable ABI,
# `std::__1::`, is unchanged across these versions -- verified empirically
# by this task's Step 1 ABI check and Step 6's full node+gtest rebuild, not
# assumed). visual_renderer's OWN toolchain
# (cmake/toolchain-clang-libcxx.cmake) is UNCHANGED -- still clang-14 -- so
# this deviation is scoped to the vcpkg/cesium build alone.

# DEVIATION #2 (recorded, root-caused empirically, VM-061 Step 1):
# VCPKG_CHAINLOAD_TOOLCHAIN_FILE REPLACES vcpkg's own stock platform
# toolchain (scripts/toolchains/linux.cmake) for the target triplet
# entirely -- it does not layer on top of it. That stock file is what
# normally derives CMAKE_SYSTEM_NAME/CMAKE_SYSTEM_PROCESSOR from
# VCPKG_TARGET_ARCHITECTURE; since our overlay triplet chainloads straight
# to THIS file instead, neither ever got set, and CMAKE_SYSTEM_PROCESSOR
# came out BLANK for every port's nested configure -- confirmed via a
# captured failing log: zlib-ng's own CMakeLists.txt has an
# `elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" AND WITH_SSE2)`, and with
# CMAKE_SYSTEM_PROCESSOR empty, CMake's if()/elseif() parses the leftover
# tokens as "given arguments: STREQUAL x86_64 AND WITH_SSE2 -- Unknown
# arguments specified", a hard configure error, not something specific to
# zlib-ng's own code. Fix: set both explicitly here, matching what
# linux.cmake would have done for VCPKG_TARGET_ARCHITECTURE=x64 (the only
# architecture this overlay triplet targets).
set(CMAKE_SYSTEM_NAME "Linux" CACHE STRING "")
set(CMAKE_SYSTEM_PROCESSOR "x86_64" CACHE STRING "")

if(DEFINED ENV{XDG_CACHE_HOME} AND NOT "$ENV{XDG_CACHE_HOME}" STREQUAL "")
    set(_mpviz_vcpkg_cache_home "$ENV{XDG_CACHE_HOME}")
else()
    set(_mpviz_vcpkg_cache_home "$ENV{HOME}/.cache")
endif()
set(_mpviz_vcpkg_cesium_prefix "${_mpviz_vcpkg_cache_home}/mpviz-toolchain-cesium")
set(_mpviz_vcpkg_real_clangxx "${_mpviz_vcpkg_cesium_prefix}/root/usr/lib/llvm-18/bin/clang++")
set(_mpviz_vcpkg_real_clang "${_mpviz_vcpkg_cesium_prefix}/root/usr/lib/llvm-18/bin/clang")
set(_mpviz_vcpkg_libdir1 "${_mpviz_vcpkg_cesium_prefix}/root/usr/lib/llvm-18/lib")
set(_mpviz_vcpkg_libdir2 "${_mpviz_vcpkg_cesium_prefix}/root/usr/lib/x86_64-linux-gnu")

if(NOT EXISTS "${_mpviz_vcpkg_real_clangxx}")
    message(FATAL_ERROR
        "visual_renderer (vcpkg overlay triplet): clang-18 not found at "
        "${_mpviz_vcpkg_real_clangxx}. Run scripts/setup_toolchain_cesium.sh "
        "first (see that file's header comment for why this is a SEPARATE, "
        "newer toolchain from the project's usual clang-14).")
endif()

# vcpkg's own internal compiler-identity probe (scripts/detect_compiler) and
# individual port builds both need: (a) the clang++ DRIVER BINARY itself to
# find its own libLLVM-18/libclang-cpp shared libs (LD_LIBRARY_PATH, since
# this is an unpacked-not-installed rootless prefix, not on the system
# loader's default path) and (b) -stdlib=libc++ applied unconditionally even
# when a caller (vcpkg-internal or a port's own build steps) supplies no
# flags of its own -- confirmed necessary the hard way (VM-061 Step 1): a
# bare compile with no flags does not reliably get our CMAKE_CXX_FLAGS_INIT.
# A tiny wrapper script that hard-codes LD_LIBRARY_PATH + -stdlib=libc++
# covers both of these in one place.
#
# DEVIATION #3 (recorded, root-caused empirically, VM-061 Step 1 retry): the
# rpath flags do NOT belong on this wrapper's compile-and-link-both exec
# line -- they were originally baked in here so vcpkg's produced test/tool
# EXECUTABLES could find libc++/libc++abi/libunwind at runtime without
# LD_LIBRARY_PATH set. But this wrapper is also invoked compile-ONLY (`-c`,
# no link), where an `-Wl,...` linker flag is a "linker input unused"
# WARNING from clang -- harmless on its own, but SILENTLY FATAL to any
# vcpkg port's own `-Werror`-guarded feature probe (e.g. blend2d's
# check_cxx_compiler_flag("-mavx2 ... -Werror" ...) SIMD detection): the
# warning-turned-error makes the probe report the flag "unsupported" even
# though it compiles fine, which starved blend2d's checksum.cpp of the
# adler32Update_SSE2/crc32Update_SSE4_2 symbols its unconditional
# `#if BL_TARGET_ARCH_X86` block still references, and failed the build --
# reproduced directly against CMakeConfigureLog.yaml (exitCode 2,
# "'linker' input unused [-Werror,-Wunused-command-line-argument]"), not a
# guess. Fix: rpath is a LINK-time concern, so it moves to
# CMAKE_EXE_LINKER_FLAGS_INIT/CMAKE_SHARED_LINKER_FLAGS_INIT below (CMake
# applies those only when actually linking), and the wrapper itself carries
# only what every invocation -- compile or link -- genuinely needs.
set(_mpviz_vcpkg_wrap_dir "${_mpviz_vcpkg_cesium_prefix}/vcpkg-wrap")
file(MAKE_DIRECTORY "${_mpviz_vcpkg_wrap_dir}")
set(_mpviz_vcpkg_wrap_clangxx "${_mpviz_vcpkg_wrap_dir}/clang++")
set(_mpviz_vcpkg_wrap_clang "${_mpviz_vcpkg_wrap_dir}/clang")
if(NOT EXISTS "${_mpviz_vcpkg_wrap_clangxx}")
    file(WRITE "${_mpviz_vcpkg_wrap_clangxx}"
"#!/bin/sh
# Generated by vcpkg-clang-libcxx-toolchain.cmake -- always libc++-18.
export LD_LIBRARY_PATH=\"${_mpviz_vcpkg_libdir1}:${_mpviz_vcpkg_libdir2}\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\"
exec \"${_mpviz_vcpkg_real_clangxx}\" -stdlib=libc++ \"\$@\"
")
    execute_process(COMMAND chmod +x "${_mpviz_vcpkg_wrap_clangxx}")
endif()
if(NOT EXISTS "${_mpviz_vcpkg_wrap_clang}")
    file(WRITE "${_mpviz_vcpkg_wrap_clang}"
"#!/bin/sh
# Generated by vcpkg-clang-libcxx-toolchain.cmake.
export LD_LIBRARY_PATH=\"${_mpviz_vcpkg_libdir1}:${_mpviz_vcpkg_libdir2}\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\"
exec \"${_mpviz_vcpkg_real_clang}\" \"\$@\"
")
    execute_process(COMMAND chmod +x "${_mpviz_vcpkg_wrap_clang}")
endif()

set(CMAKE_C_COMPILER "${_mpviz_vcpkg_wrap_clang}" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_mpviz_vcpkg_wrap_clangxx}" CACHE FILEPATH "" FORCE)

# NOTE: no -nostdlib++ here (unlike this project's own
# toolchain-clang-libcxx.cmake) -- that convention pairs with visual_renderer
# manually adding explicit static libc++/libc++abi/libunwind archives to ITS
# OWN link line. Individual vcpkg ports build their own internal
# executables/tools with ordinary upstream CMakeLists.txt that know nothing
# of that convention; -nostdlib++ there would silently drop the C++ runtime
# from every such link. Plain dynamic -stdlib=libc++ (this prefix ships the
# matching libc++.so/.so.1 + libc++abi + libunwind .so's) is what upstream
# ports actually expect.
set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
# rpath belongs here (link-time only) -- see DEVIATION #3 above for why it
# must not sit on the wrapper's compile-and-link exec line.
#
# DEVIATION #4 (recorded, root-caused empirically, VM-061 Step 3/6 runtime):
# plain -Wl,-rpath alone left every test binary dying at RUN time with
# "libunwind.so.1: cannot open shared object file" even though the rpath IS
# on the executable (confirmed via readelf -d) and libunwind.so.1 genuinely
# exists at that path. Root cause: this rootless clang-18 prefix's
# libc++.so.1 itself carries NO rpath/runpath of its own (readelf -d
# libc++.so.1: NEEDED libunwind.so.1/libc++abi.so.1, no RPATH/RUNPATH
# entries) -- with modern ld's default DT_RUNPATH tagging, an executable's
# own runpath is consulted only for ITS direct NEEDED entries, not
# transitively for a dependency's OWN dependencies, so libc++.so.1's need
# for libunwind.so.1 falls through to the plain system search path (not on
# it, since this is an unpacked-not-installed prefix) and fails.
# --disable-new-dtags tags the binary DT_RPATH instead, which (per the
# dynamic linker's documented legacy behavior) IS consulted transitively --
# the simplest fix that needs no LD_LIBRARY_PATH at run time, matching how
# ctest/gtest_discover_tests and colcon later invoke these binaries plainly.
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-stdlib=libc++ -Wl,--disable-new-dtags -Wl,-rpath,${_mpviz_vcpkg_libdir1} -Wl,-rpath,${_mpviz_vcpkg_libdir2}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT
    "-stdlib=libc++ -Wl,--disable-new-dtags -Wl,-rpath,${_mpviz_vcpkg_libdir1} -Wl,-rpath,${_mpviz_vcpkg_libdir2}")
