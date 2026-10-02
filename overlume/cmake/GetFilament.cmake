# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Produces: FILAMENT_ROOT (has include/), FILAMENT_LIB_DIR (the *.a),
# FILAMENT_HOST_MATC (a matc that runs on the build host), target
# Filament::filament. Either the prebuilt Linux x86_64 SDK or Filament built
# from source (OVERLUME_FILAMENT_FROM_SOURCE).

set(FILAMENT_VERSION "1.56.5")
set(FILAMENT_URL
    "https://github.com/google/filament/releases/download/v${FILAMENT_VERSION}/filament-v${FILAMENT_VERSION}-linux.tgz")
set(FILAMENT_SHA256
    "b74e2a81b64fe06171a50f27e5a7a07afe7b8ac24bea36558fbca8b013466a31")
set(FILAMENT_SRC_URL
    "https://github.com/google/filament/archive/refs/tags/v${FILAMENT_VERSION}.tar.gz")
set(FILAMENT_SRC_SHA256
    "352cdef7fb8c4652d8e3b8d5301435ca04e045dc9b2a90496aed2a40ecaee73e")

# A prebuilt SDK exists only for Linux x86_64 (mac/windows-x64/ios prebuilts
# are wired in by their own tasks); everything else builds from source.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(_overlume_filament_src_default OFF)
else()
    set(_overlume_filament_src_default ON)
endif()
option(OVERLUME_FILAMENT_FROM_SOURCE
    "Build Filament ${FILAMENT_VERSION} from source with this build's compilers instead of using the prebuilt SDK"
    ${_overlume_filament_src_default})

function(_overlume_fetch file_url file_sha out_path)
    if(EXISTS "${out_path}")
        return()
    endif()
    message(STATUS "overlume: fetching ${file_url} ...")
    file(DOWNLOAD "${file_url}" "${out_path}.part"
         EXPECTED_HASH SHA256=${file_sha}
         SHOW_PROGRESS
         STATUS _dl_status)
    list(GET _dl_status 0 _dl_code)
    if(NOT _dl_code EQUAL 0)
        list(GET _dl_status 1 _dl_msg)
        file(REMOVE "${out_path}.part")
        message(FATAL_ERROR "Failed to download ${file_url}: ${_dl_msg}")
    endif()
    file(RENAME "${out_path}.part" "${out_path}")
endfunction()

if(OVERLUME_FILAMENT_FROM_SOURCE)
    set(_fil_prefix "${CMAKE_BINARY_DIR}/_deps/filament-src-install")
    set(_fil_stamp "${_fil_prefix}/.overlume-filament-${FILAMENT_VERSION}.stamp")
    set(_fil_log "${CMAKE_BINARY_DIR}/_deps/filament-src-build.log")
    if(NOT EXISTS "${_fil_stamp}")
        set(_fil_tarball "${CMAKE_BINARY_DIR}/_deps/filament-v${FILAMENT_VERSION}-src.tar.gz")
        set(_fil_src "${CMAKE_BINARY_DIR}/_deps/filament-src")
        set(_fil_bld "${CMAKE_BINARY_DIR}/_deps/filament-src-build")
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
        _overlume_fetch("${FILAMENT_SRC_URL}" "${FILAMENT_SRC_SHA256}" "${_fil_tarball}")
        if(NOT EXISTS "${_fil_src}/CMakeLists.txt")
            file(MAKE_DIRECTORY "${_fil_src}")
            file(ARCHIVE_EXTRACT INPUT "${_fil_tarball}" DESTINATION "${_fil_src}")
            file(GLOB _fil_top "${_fil_src}/filament-*")
            list(GET _fil_top 0 _fil_top)
            file(RENAME "${_fil_top}" "${_fil_src}.tmp")
            file(REMOVE_RECURSE "${_fil_src}")
            file(RENAME "${_fil_src}.tmp" "${_fil_src}")
        endif()

        set(_fil_args
            -G Ninja
            -DCMAKE_BUILD_TYPE=Release
            -DCMAKE_INSTALL_PREFIX=${_fil_prefix}
            -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
            -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
            "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
            "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
            "-DCMAKE_SHARED_LINKER_FLAGS=${CMAKE_SHARED_LINKER_FLAGS}"
            -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DFILAMENT_SKIP_SAMPLES=ON
            -DFILAMENT_SUPPORTS_VULKAN=OFF
            -DFILAMENT_ENABLE_JAVA=OFF)
        if(CMAKE_TOOLCHAIN_FILE AND CMAKE_CROSSCOMPILING)
            # Cross targets (Android, iOS, Windows arm64): forward the toolchain
            # and ABI variables; matc must come from a host prebuilt.
            list(APPEND _fil_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
            foreach(_v ANDROID_ABI ANDROID_PLATFORM ANDROID_STL CMAKE_SYSTEM_NAME
                       CMAKE_OSX_ARCHITECTURES CMAKE_OSX_DEPLOYMENT_TARGET
                       CMAKE_ANDROID_NDK CMAKE_SYSTEM_VERSION)
                if(DEFINED ${_v})
                    list(APPEND _fil_args "-D${_v}=${${_v}}")
                endif()
            endforeach()
            if(NOT FILAMENT_HOST_MATC)
                message(FATAL_ERROR "overlume: cross-building Filament needs "
                    "-DFILAMENT_HOST_MATC=<host matc> (from the host prebuilt SDK).")
            endif()
        endif()

        message(STATUS "overlume: building Filament ${FILAMENT_VERSION} from source "
                       "(long; log: ${_fil_log}) ...")
        file(REMOVE "${_fil_log}")
        include(ProcessorCount)
        ProcessorCount(_fil_jobs)
        foreach(_step
                "configure;${CMAKE_COMMAND};-S;${_fil_src};-B;${_fil_bld};${_fil_args}"
                "build;${CMAKE_COMMAND};--build;${_fil_bld};--target;install;-j;${_fil_jobs}")
            list(POP_FRONT _step _step_name)
            execute_process(COMMAND ${_step}
                RESULT_VARIABLE _step_rc
                OUTPUT_VARIABLE _step_out ERROR_VARIABLE _step_out)
            file(APPEND "${_fil_log}" "=== ${_step_name} ===\n${_step_out}\n")
            if(NOT _step_rc EQUAL 0)
                message(FATAL_ERROR "overlume: Filament source ${_step_name} failed "
                        "(rc=${_step_rc}); see ${_fil_log}")
            endif()
        endforeach()
        file(WRITE "${_fil_stamp}" "${FILAMENT_VERSION}\n")
    endif()
    set(FILAMENT_ROOT "${_fil_prefix}")
    if(NOT FILAMENT_HOST_MATC)
        set(FILAMENT_HOST_MATC "${FILAMENT_ROOT}/bin/matc")
    endif()
else()
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
    set(FILAMENT_HOST_MATC "${FILAMENT_ROOT}/bin/matc")
endif()

if(NOT EXISTS "${FILAMENT_ROOT}/include/filament/Engine.h")
    message(FATAL_ERROR "Filament SDK not found at ${FILAMENT_ROOT} after fetch/extract.")
endif()

# lib/<arch>/ holds every static archive (arch dir name differs per target).
file(GLOB _filament_arch_dirs LIST_DIRECTORIES true "${FILAMENT_ROOT}/lib/*")
foreach(_d ${_filament_arch_dirs})
    if(IS_DIRECTORY "${_d}")
        set(FILAMENT_LIB_DIR "${_d}")
        break()
    endif()
endforeach()
file(GLOB _filament_static_libs "${FILAMENT_LIB_DIR}/*.a")

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
