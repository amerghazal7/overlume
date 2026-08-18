# toolchain-clang-libcxx.cmake — selects the clang-14/libc++ toolchain
# visual_renderer must build with (spec §2: this lib is clang/libc++, the
# gcc ROS node stays gcc/libstdc++, and the two must never mix at an ABI
# boundary — see include/visual_renderer/api.h).
#
# Usage:
#   scripts/setup_toolchain.sh   # once, bootstraps the fallback prefix below
#   cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .
#   cmake --build build
#   ctest --test-dir build
#
# Resolution order for CMAKE_CXX_COMPILER:
#   1. A clang++ already on PATH, but only if it actually has a co-located
#      libc++ static archive (`clang++ -stdlib=libc++ -print-file-name=libc++.a`
#      resolves to a real file) — so a box with a real (root-installed)
#      clang+libc++-dev setup just works without the rootless prefix.
#   2. scripts/setup_toolchain.sh's rootless bootstrap prefix
#      (${XDG_CACHE_HOME:-$HOME/.cache}/mpviz-toolchain/bin/clang++).
#
# This also bakes in -stdlib=libc++ for compile and link so the documented
# configure line above needs no other flags.

if(DEFINED ENV{XDG_CACHE_HOME} AND NOT "$ENV{XDG_CACHE_HOME}" STREQUAL "")
    set(_mpviz_cache_home "$ENV{XDG_CACHE_HOME}")
else()
    set(_mpviz_cache_home "$ENV{HOME}/.cache")
endif()
set(_mpviz_toolchain_prefix "${_mpviz_cache_home}/mpviz-toolchain")

find_program(_mpviz_path_clangxx NAMES clang++)
set(_mpviz_chosen_clangxx "")
if(_mpviz_path_clangxx)
    execute_process(
        COMMAND "${_mpviz_path_clangxx}" -stdlib=libc++ -print-file-name=libc++.a
        OUTPUT_VARIABLE _mpviz_path_libcxx_a
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(_mpviz_path_libcxx_a AND EXISTS "${_mpviz_path_libcxx_a}")
        set(_mpviz_chosen_clangxx "${_mpviz_path_clangxx}")
    endif()
endif()

if(NOT _mpviz_chosen_clangxx)
    set(_mpviz_prefix_clangxx "${_mpviz_toolchain_prefix}/bin/clang++")
    if(NOT EXISTS "${_mpviz_prefix_clangxx}")
        message(FATAL_ERROR
            "visual_renderer: no clang++ on PATH with a co-located libc++, and "
            "the rootless toolchain prefix (${_mpviz_prefix_clangxx}) doesn't "
            "exist yet. Run scripts/setup_toolchain.sh first.")
    endif()
    set(_mpviz_chosen_clangxx "${_mpviz_prefix_clangxx}")
endif()

set(CMAKE_CXX_COMPILER "${_mpviz_chosen_clangxx}" CACHE FILEPATH
    "clang++ (libc++) for visual_renderer" FORCE)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
# -nostdlib++: visual_renderer links libc++/libc++abi/libunwind as explicit
# static archives (see CMakeLists.txt) instead of the driver's default
# dynamic -lc++, so the result needs no LD_LIBRARY_PATH/rpath to run.
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
