# GetFilament.cmake — fetches the pinned Google Filament prebuilt Linux SDK
# and exposes it as the imported target `Filament::filament`.
#
# Pin (Epic 0, Task 1 — see plan docs/superpowers/plans/2026-08-18-visual-mode.md):
#   FILAMENT_VERSION = 1.75.0  (latest stable release with prebuilt Linux
#   binaries at pin time: https://github.com/google/filament/releases).
#   The release is built with clang + libc++ (confirmed via `std::__1::`
#   symbol mangling in the archives), matching this directory's enforced
#   toolchain.
#
# Fallback (per spec §2, "known risk, handled by design"): if the prebuilt
# clang/libc++ static-link route proves brittle in practice, build Filament
# from source instead (https://github.com/google/filament, `build.sh`) with
# either clang/libc++ (preferred, matches upstream CI) or gcc/libstdc++ (also
# supported by upstream, less tested) and point FILAMENT_ROOT below at that
# source build's `out/release/filament` install directory — the POD boundary
# in include/visual_renderer/api.h makes either choice invisible to the ROS
# node, so no other file needs to change.

set(FILAMENT_VERSION "1.75.0")
set(FILAMENT_URL
    "https://github.com/google/filament/releases/download/v${FILAMENT_VERSION}/filament-v${FILAMENT_VERSION}-linux.tgz")
set(FILAMENT_SHA256
    "c5d2e0f692e5fb98ed029a5a3a52c8174660d02844d5db5a804dc5264bbab6d1")

set(FILAMENT_FETCH_ROOT "${CMAKE_BINARY_DIR}/_deps/filament-${FILAMENT_VERSION}"
    CACHE PATH "Where the prebuilt Filament SDK is unpacked")
set(FILAMENT_ROOT "${FILAMENT_FETCH_ROOT}/filament"
    CACHE PATH "Filament SDK root (contains include/ and lib/x86_64/)")

if(NOT EXISTS "${FILAMENT_ROOT}/include/filament/Engine.h")
    set(_filament_tarball "${CMAKE_BINARY_DIR}/_deps/filament-v${FILAMENT_VERSION}-linux.tgz")
    message(STATUS "visual_renderer: fetching Filament ${FILAMENT_VERSION} prebuilt Linux SDK ...")
    file(DOWNLOAD "${FILAMENT_URL}" "${_filament_tarball}"
         EXPECTED_HASH SHA256=${FILAMENT_SHA256}
         SHOW_PROGRESS
         STATUS _filament_dl_status)
    list(GET _filament_dl_status 0 _filament_dl_code)
    if(NOT _filament_dl_code EQUAL 0)
        list(GET _filament_dl_status 1 _filament_dl_msg)
        message(FATAL_ERROR "Failed to download Filament SDK: ${_filament_dl_msg}")
    endif()
    file(MAKE_DIRECTORY "${FILAMENT_FETCH_ROOT}")
    file(ARCHIVE_EXTRACT INPUT "${_filament_tarball}" DESTINATION "${FILAMENT_FETCH_ROOT}")
endif()

if(NOT EXISTS "${FILAMENT_ROOT}/include/filament/Engine.h")
    message(FATAL_ERROR "Filament SDK not found at ${FILAMENT_ROOT} after fetch/extract.")
endif()

file(GLOB _filament_static_libs "${FILAMENT_ROOT}/lib/x86_64/*.a")

add_library(Filament::filament INTERFACE IMPORTED)
target_include_directories(Filament::filament INTERFACE "${FILAMENT_ROOT}/include")
# Whole archive linkage of everything in lib/x86_64/: the individual static
# libs cross-reference each other and Filament ships no CMake package config
# to resolve the graph precisely. Narrow this to the specific archives a
# consumer needs (e.g. filament, backend, utils, filabridge, filaflat,
# smol-v) if link time or binary size becomes a problem.
target_link_libraries(Filament::filament INTERFACE
    ${_filament_static_libs}
    EGL
    GLESv2
    dl
    pthread
)
