# GetFilament.cmake — fetches the pinned Google Filament prebuilt Linux SDK
# and exposes it as the imported target `Filament::filament`.
#
# Pin (Epic 0, Task 1 — see plan docs/superpowers/plans/2026-08-18-visual-mode.md):
#   FILAMENT_VERSION = 1.56.5 (https://github.com/google/filament/releases).
#   The release is built with clang + libc++ (confirmed via `std::__1::`
#   symbol mangling in the archives), matching this directory's enforced
#   toolchain.
#
#   NOT anything newer: starting around v1.57.0-v1.60.0, Filament's Linux CI
#   moved to a host glibc >=2.38, so several archives (libfilament, libutils,
#   libbackend, ...) reference `__isoc23_sscanf` (glibc's ISO-C23-conformant
#   sscanf, only exported starting glibc 2.38 — Ubuntu 23.10+). That's
#   unresolvable on this project's Ubuntu 22.04 (glibc 2.35) dev/robot boxes —
#   confirmed by actually linking smoke/filament_link_probe against v1.75.0
#   and v1.74.1 (undefined reference to `__isoc23_sscanf`). The dependency on
#   host glibc isn't monotonic across every patch release (some later patch
#   tags were apparently rebuilt/backported on an older image again — e.g.
#   v1.56.5 is clean but v1.56.8 is not), so this pin was chosen by bisecting
#   `strings lib/x86_64/*.a | grep isoc23` across releases down to a version
#   confirmed both symbol-clean AND a real link+run of
#   smoke/filament_link_probe. Re-verify both before ever bumping
#   FILAMENT_VERSION — don't assume newer-is-safer.
#
# Fallback (per spec §2, "known risk, handled by design"): if the prebuilt
# clang/libc++ static-link route proves brittle in practice, build Filament
# from source instead (https://github.com/google/filament, `build.sh`) with
# either clang/libc++ (preferred, matches upstream CI) or gcc/libstdc++ (also
# supported by upstream, less tested) and point FILAMENT_ROOT below at that
# source build's `out/release/filament` install directory — the POD boundary
# in include/visual_renderer/api.h makes either choice invisible to the ROS
# node, so no other file needs to change.

set(FILAMENT_VERSION "1.56.5")
set(FILAMENT_URL
    "https://github.com/google/filament/releases/download/v${FILAMENT_VERSION}/filament-v${FILAMENT_VERSION}-linux.tgz")
set(FILAMENT_SHA256
    "b74e2a81b64fe06171a50f27e5a7a07afe7b8ac24bea36558fbca8b013466a31")

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

# Filament's ~30 archives (filament <-> backend <-> utils <-> filabridge <->
# filaflat <-> smol-v, ...) mutually cross-reference each other and ship no
# CMake package config to resolve the graph precisely. A flat file-path list
# handed to target_link_libraries is NOT whole-archive linkage — CMake only
# auto-repeats *targets* for cyclic dependencies between CMake targets, never
# raw archive paths — so ld resolves the list once, left to right, and a
# forward reference from an earlier archive to a symbol defined in a later
# one is an undefined-symbol link error (confirmed: this failed to link
# before this fix — see smoke/filament_link_probe.cpp).
#
# `-Wl,--start-group ... --end-group` tells the linker to keep re-scanning
# the enclosed archives until no new undefined symbols are resolved, which is
# exactly "whole archive" behavior for a cyclic archive graph. GNU ld, gold,
# and lld (Filament's own upstream Linux CI toolchain) all support it.
# Narrow this to the specific archives a consumer needs (e.g. filament,
# backend, utils, filabridge, filaflat, smol-v) if link time or binary size
# becomes a problem.
target_link_libraries(Filament::filament INTERFACE
    -Wl,--start-group
    ${_filament_static_libs}
    -Wl,--end-group
    EGL
    GLESv2
    dl
    pthread
)
