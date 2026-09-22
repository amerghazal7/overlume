# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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

get_filename_component(_overlume_toolchain_bindir "${_overlume_chosen_clangxx}" DIRECTORY)
set(CMAKE_C_COMPILER "${_overlume_toolchain_bindir}/clang" CACHE FILEPATH
    "clang (matching CMAKE_CXX_COMPILER) for overlume's C deps" FORCE)

set(CMAKE_CXX_FLAGS_INIT "-stdlib=libc++")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-stdlib=libc++ -nostdlib++")
