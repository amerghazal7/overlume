# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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
    message(STATUS "overlume: fetching Filament ${FILAMENT_VERSION} prebuilt Linux SDK ...")
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

target_link_libraries(Filament::filament INTERFACE
    -Wl,--start-group
    ${_filament_static_libs}
    -Wl,--end-group
    EGL
    GLESv2
    dl
    pthread
)
