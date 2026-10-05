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

# The macOS SDK ships arm64 libraries and arm64 host tools only. Apple builds always fetch it for
# its tools (matc, ...); macOS arm64 also links its libraries.
set(FILAMENT_MAC_URL
    "https://github.com/google/filament/releases/download/v${FILAMENT_VERSION}/filament-v${FILAMENT_VERSION}-mac.tgz")
set(FILAMENT_MAC_SHA256
    "412f135d48683e80f1109e2a75ce56d4b8e94174a7396a2e79203e7c03597a5b")

# The Windows SDK (note: not versioned in its file name, and no top-level directory) holds /MD
# static libraries for x64 only (lib/x86_64/md/*.lib); Windows arm64 builds from source.
set(FILAMENT_WIN_URL
    "https://github.com/google/filament/releases/download/v${FILAMENT_VERSION}/filament-windows.tgz")
set(FILAMENT_WIN_SHA256
    "b58bb67e41fc1ca088d3bbdb66a2ae232c7355bba11e0825e27cd5732d2cef0a")

set(_overlume_apple_arch "${CMAKE_SYSTEM_PROCESSOR}")
if(APPLE AND CMAKE_OSX_ARCHITECTURES)
    list(GET CMAKE_OSX_ARCHITECTURES 0 _overlume_apple_arch)
endif()
# Prebuilt SDKs: Linux x86_64, macOS arm64 and Windows x64. The iOS SDK has no arm64 simulator slice and the mac
# SDK no x86_64 one, so everything else (Linux aarch64, Android, iOS, macOS x86_64, Windows arm64)
# builds from source.
if((CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64") OR
   (CMAKE_SYSTEM_NAME STREQUAL "Darwin" AND _overlume_apple_arch STREQUAL "arm64") OR
   (WIN32 AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|x86_64)$"))
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

# One logged step of the source build; FATAL_ERROR names the log on failure.
function(_overlume_fil_run step_name log)
    execute_process(COMMAND ${ARGN}
        RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _out)
    file(APPEND "${log}" "=== ${step_name} ===\n${_out}\n")
    if(NOT _rc EQUAL 0)
        # ponytail: matc (glslang) segfaults now and then under a heavily parallel material build;
        # every step is resumable (ninja/cmake), so one retry is the whole fix.
        execute_process(COMMAND ${ARGN}
            RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _out)
        file(APPEND "${log}" "=== ${step_name} (retry) ===\n${_out}\n")
    endif()
    if(NOT _rc EQUAL 0)
        string(LENGTH "${_out}" _out_len)
        math(EXPR _out_from "${_out_len} - 6000")
        if(_out_from LESS 0)
            set(_out_from 0)
        endif()
        string(SUBSTRING "${_out}" ${_out_from} -1 _out_tail)
        message(FATAL_ERROR "overlume: Filament source ${step_name} failed "
                "(rc=${_rc}); see ${log}. Last output:\n${_out_tail}")
    endif()
endfunction()

set(_overlume_exe_suffix "")
if(CMAKE_HOST_WIN32)
    set(_overlume_exe_suffix ".exe")
endif()

if(APPLE)
    if(NOT CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin" OR NOT CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        message(FATAL_ERROR "overlume: Apple builds need an Apple Silicon macOS host "
                "(Filament's prebuilt host tools are arm64).")
    endif()
    set(FILAMENT_MAC_ROOT "${CMAKE_BINARY_DIR}/_deps/filament-mac-${FILAMENT_VERSION}/filament")
    if(NOT EXISTS "${FILAMENT_MAC_ROOT}/bin/matc")
        set(_fil_mac_tarball "${CMAKE_BINARY_DIR}/_deps/filament-v${FILAMENT_VERSION}-mac.tgz")
        file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
        _overlume_fetch("${FILAMENT_MAC_URL}" "${FILAMENT_MAC_SHA256}" "${_fil_mac_tarball}")
        file(ARCHIVE_EXTRACT INPUT "${_fil_mac_tarball}"
             DESTINATION "${CMAKE_BINARY_DIR}/_deps/filament-mac-${FILAMENT_VERSION}")
    endif()
    # matc runs on this host whatever the target.
    set(FILAMENT_HOST_MATC "${FILAMENT_MAC_ROOT}/bin/matc")
endif()

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

        if(APPLE)
            # Filament 1.56.5 builds -Werror with -Wdeprecated after any flag we could add, and
            # Xcode 15's SDK deprecates the Metal calls it uses: drop -Werror from its own targets.
            foreach(_f filament/CMakeLists.txt filament/backend/CMakeLists.txt libs/gltfio/CMakeLists.txt)
                file(READ "${_fil_src}/${_f}" _fc)
                string(REPLACE "-Werror\n" "\n" _fc "${_fc}")
                string(REPLACE "-Wall -Werror)" "-Wall)" _fc "${_fc}")
                file(WRITE "${_fil_src}/${_f}" "${_fc}")
            endforeach()
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
        if(ANDROID)
            # 1.56.5 only compiles PlatformEGL.cpp (base of PlatformEGLAndroid) when EGL is set,
            # and only sets it for Linux; without it libbackend.a has unresolved PlatformEGL symbols.
            list(APPEND _fil_args -DEGL=ON)
            # The NDK toolchain adds -g to every compile; the static package would carry it.
            list(APPEND _fil_args "-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG -g0"
                                  "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0")
        endif()
        # This project enables CXX only: with no C compiler of its own, let Filament find one.
        list(FILTER _fil_args EXCLUDE REGEX "^-DCMAKE_C_COMPILER=$")
        if(WIN32)
            # MSVC cl for both languages (Filament rejects clang on Windows), /MD like the rest of the build. This project's own
            # CMAKE_CXX_FLAGS / linker flags are MSVC defaults Filament sets for itself.
            list(FILTER _fil_args EXCLUDE REGEX "^-DCMAKE_(CXX_FLAGS|EXE_LINKER_FLAGS|SHARED_LINKER_FLAGS)=")
            list(APPEND _fil_args "-DCMAKE_C_COMPILER=${CMAKE_CXX_COMPILER}" -DUSE_STATIC_CRT=OFF
                 "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL")
        endif()
        set(_fil_host_tools OFF)
        if(APPLE)
            # Metal only. Filament's own iOS toolchain (-DIOS=1) takes the compilers and SDK from
            # xcrun; its cross build imports the host tools from <src>/out/, which here are the
            # arm64 tools of the prebuilt mac SDK (no second, native Filament build).
            list(APPEND _fil_args -DFILAMENT_SUPPORTS_OPENGL=OFF -DFILAMENT_SUPPORTS_METAL=ON
                 "-DCMAKE_OSX_ARCHITECTURES=${_overlume_apple_arch}"
                 "-DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}")
            if(IOS)
                if(CMAKE_OSX_SYSROOT MATCHES "[Ss]imulator")
                    set(_fil_ios_platform iphonesimulator)
                else()
                    set(_fil_ios_platform iphoneos)
                endif()
                # Filament's toolchain tags every simulator object as device iOS (-mios-version-min), which
                # ld64 rejects ("building for 'iOS-simulator', but linking in object file built for 'iOS'").
                if(_fil_ios_platform STREQUAL "iphonesimulator")
                    file(READ "${_fil_src}/third_party/clang/iOS.cmake" _fc)
                    string(REPLACE "SET(IOS_COMMON_FLAGS \"-m\${PLATFORM_FLAG_NAME}-version-min=\${IOS_MIN_TARGET}\")"
                        "SET(IOS_COMMON_FLAGS \"-mios-simulator-version-min=\${IOS_MIN_TARGET}\")" _fc_patched "${_fc}")
                    if(_fc_patched STREQUAL _fc AND NOT _fc MATCHES "-mios-simulator-version-min")
                        message(FATAL_ERROR "overlume: Filament's iOS.cmake no longer has the line this patch targets")
                    endif()
                    set(_fc "${_fc_patched}")
                    file(WRITE "${_fil_src}/third_party/clang/iOS.cmake" "${_fc}")
                endif()
                list(FILTER _fil_args EXCLUDE REGEX "^-DCMAKE_(C|CXX)_COMPILER=|^-DCMAKE_OSX_|^-DCMAKE_(C|CXX)_FLAGS=")
                # Minimum OS for every language Filament compiles; without it clang defaults the C++/ObjC++
                # objects to the SDK version (17.5), above the pinned deployment target.
                if(_fil_ios_platform STREQUAL "iphonesimulator")
                    set(_fil_min "-mios-simulator-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}")
                else()
                    set(_fil_min "-mios-version-min=${CMAKE_OSX_DEPLOYMENT_TARGET}")
                endif()
                list(APPEND _fil_args "-DCMAKE_C_FLAGS=${_fil_min}" "-DCMAKE_CXX_FLAGS=${_fil_min}"
                     "-DCMAKE_ASM_FLAGS=${_fil_min}" -DIOS=1 "-DIOS_ARCH=${_overlume_apple_arch}"
                     -DPLATFORM_NAME=${_fil_ios_platform} -DIMPORT_EXECUTABLES_DIR=out
                     "-DCMAKE_TOOLCHAIN_FILE=${_fil_src}/third_party/clang/iOS.cmake")
                set(_fil_import "# Generated by overlume/cmake/GetFilament.cmake: the prebuilt mac SDK's host tools.\n")
                foreach(_t matc cmgen filamesh mipgen resgen uberz glslminifier)
                    string(APPEND _fil_import
                        "add_executable(${_t} IMPORTED)\n"
                        "set_target_properties(${_t} PROPERTIES IMPORTED_LOCATION_RELEASE \"${FILAMENT_MAC_ROOT}/bin/${_t}\")\n")
                endforeach()
                file(WRITE "${_fil_src}/out/ImportExecutables-Release.cmake" "${_fil_import}")
            endif()
        elseif(CMAKE_TOOLCHAIN_FILE AND CMAKE_CROSSCOMPILING)
            # Cross targets (Linux aarch64, Android, iOS, Windows arm64): forward
            # the toolchain and ABI variables. Filament's cross build includes
            # ImportExecutables-Release.cmake from its source tree (written by a
            # native build's configure) and needs a matc that runs on this host.
            list(APPEND _fil_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
            foreach(_v ANDROID_ABI ANDROID_PLATFORM ANDROID_STL CMAKE_SYSTEM_NAME
                       CMAKE_SYSTEM_PROCESSOR CMAKE_OSX_ARCHITECTURES CMAKE_OSX_DEPLOYMENT_TARGET
                       CMAKE_ANDROID_NDK CMAKE_SYSTEM_VERSION)
                if(DEFINED ${_v})
                    list(APPEND _fil_args "-D${_v}=${${_v}}")
                endif()
            endforeach()
            if(NOT FILAMENT_HOST_MATC)
                # Build the host tools from the same source tree with the host
                # toolchain (the release container's x86_64 LLVM).
                if(NOT OVERLUME_FILAMENT_HOST_TOOLCHAIN AND (OVERLUME_RELEASE_TRIPLE OR ANDROID))
                    # Android: the host tools use the same libc++ toolchain as the
                    # Linux builds (release tarball if OVERLUME_LLVM_ROOT, else the dev one).
                    if(OVERLUME_RELEASE_TRIPLE OR NOT "$ENV{OVERLUME_LLVM_ROOT}" STREQUAL "")
                        set(OVERLUME_FILAMENT_HOST_TOOLCHAIN
                            "${CMAKE_CURRENT_LIST_DIR}/toolchain-llvm-release.cmake")
                    else()
                        set(OVERLUME_FILAMENT_HOST_TOOLCHAIN
                            "${CMAKE_CURRENT_LIST_DIR}/toolchain-clang-libcxx.cmake")
                    endif()
                endif()
                if(NOT OVERLUME_FILAMENT_HOST_TOOLCHAIN)
                    message(FATAL_ERROR "overlume: cross-building Filament needs "
                        "-DFILAMENT_HOST_MATC=<host matc> or "
                        "-DOVERLUME_FILAMENT_HOST_TOOLCHAIN=<toolchain file for the build host>.")
                endif()
                set(_fil_host_bld "${CMAKE_BINARY_DIR}/_deps/filament-host-build")
                set(_fil_host_tools ON)
                set(_fil_host_args
                    -G Ninja
                    -DCMAKE_BUILD_TYPE=Release
                    -DCMAKE_TOOLCHAIN_FILE=${OVERLUME_FILAMENT_HOST_TOOLCHAIN}
                    -DFILAMENT_SKIP_SAMPLES=ON
                    -DFILAMENT_SUPPORTS_VULKAN=OFF
                    -DFILAMENT_ENABLE_JAVA=OFF)
            endif()
        endif()

        message(STATUS "overlume: building Filament ${FILAMENT_VERSION} from source "
                       "(long; log: ${_fil_log}) ...")
        file(REMOVE "${_fil_log}")
        include(ProcessorCount)
        ProcessorCount(_fil_jobs)
        if(_fil_host_tools)
            _overlume_fil_run(host-configure "${_fil_log}"
                ${CMAKE_COMMAND} -S ${_fil_src} -B ${_fil_host_bld} ${_fil_host_args})
            _overlume_fil_run(host-build "${_fil_log}"
                ${CMAKE_COMMAND} --build ${_fil_host_bld} --target
                matc cmgen filamesh mipgen resgen uberz glslminifier)
        endif()
        _overlume_fil_run(configure "${_fil_log}"
            ${CMAKE_COMMAND} -S ${_fil_src} -B ${_fil_bld} ${_fil_args})
        if(APPLE)
            # Keep going past Filament's own test executables (backend_test_mac, test_filamat do not
            # link for every slice and are not needed); the install step below fails loudly if a
            # library we do need was not built.
            execute_process(COMMAND ${CMAKE_COMMAND} --build ${_fil_bld} -j ${_fil_jobs} -- -k 0
                RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _out)
            file(APPEND "${_fil_log}" "=== build (keep going, rc=${_rc}) ===\n${_out}\n")
            _overlume_fil_run(install "${_fil_log}" ${CMAKE_COMMAND} --install ${_fil_bld})
        else()
            _overlume_fil_run(build "${_fil_log}"
                ${CMAKE_COMMAND} --build ${_fil_bld} --target install -j ${_fil_jobs})
        endif()
        file(WRITE "${_fil_stamp}" "${FILAMENT_VERSION}\n")
    endif()
    set(FILAMENT_ROOT "${_fil_prefix}")
    if(NOT FILAMENT_HOST_MATC)
        if(CMAKE_CROSSCOMPILING)
            set(FILAMENT_HOST_MATC "${CMAKE_BINARY_DIR}/_deps/filament-host-build/tools/matc/matc")
        else()
            set(FILAMENT_HOST_MATC "${FILAMENT_ROOT}/bin/matc${_overlume_exe_suffix}")
        endif()
    endif()
else()
    if(WIN32)
        set(FILAMENT_ROOT "${CMAKE_BINARY_DIR}/_deps/filament-win-${FILAMENT_VERSION}")
        if(NOT EXISTS "${FILAMENT_ROOT}/include/filament/Engine.h")
            set(_fil_win_tarball "${CMAKE_BINARY_DIR}/_deps/filament-v${FILAMENT_VERSION}-windows.tgz")
            file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
            _overlume_fetch("${FILAMENT_WIN_URL}" "${FILAMENT_WIN_SHA256}" "${_fil_win_tarball}")
            file(ARCHIVE_EXTRACT INPUT "${_fil_win_tarball}" DESTINATION "${FILAMENT_ROOT}")
        endif()
        set(FILAMENT_HOST_MATC "${FILAMENT_ROOT}/bin/matc.exe")
    elseif(APPLE)
        set(FILAMENT_ROOT "${FILAMENT_MAC_ROOT}")  # fetched above (arm64 libraries + tools)
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
endif()

if(NOT EXISTS "${FILAMENT_ROOT}/include/filament/Engine.h")
    message(FATAL_ERROR "Filament SDK not found at ${FILAMENT_ROOT} after fetch/extract.")
endif()

# lib/<arch>/ holds every static archive (arch dir name differs per target; on Windows the CRT
# flavour is one level further down: lib/x86_64/md, or lib/arm64[/md] from a source build).
if(WIN32)
    file(GLOB_RECURSE _filament_probe "${FILAMENT_ROOT}/lib/filament.lib")
    list(FILTER _filament_probe EXCLUDE REGEX "/mt/")
    list(GET _filament_probe 0 _filament_probe)
    get_filename_component(FILAMENT_LIB_DIR "${_filament_probe}" DIRECTORY)
    file(GLOB _filament_static_libs "${FILAMENT_LIB_DIR}/*.lib")
else()
    file(GLOB _filament_arch_dirs LIST_DIRECTORIES true "${FILAMENT_ROOT}/lib/*")
    foreach(_d ${_filament_arch_dirs})
        if(IS_DIRECTORY "${_d}")
            set(FILAMENT_LIB_DIR "${_d}")
            break()
        endif()
    endforeach()
    file(GLOB _filament_static_libs "${FILAMENT_LIB_DIR}/*.a")
endif()

add_library(Filament::filament INTERFACE IMPORTED)
target_include_directories(Filament::filament INTERFACE "${FILAMENT_ROOT}/include")

if(APPLE)
    # ld64 has no --start-group (it rescans archives) and the Metal back end needs frameworks.
    # Cocoa/OpenGL: the prebuilt macOS archives still reference the NSOpenGL platform.
    set(_filament_sys_libs "-framework Metal" "-framework QuartzCore" "-framework CoreVideo"
                           "-framework IOSurface" "-framework Foundation")
    if(IOS)
        list(APPEND _filament_sys_libs "-framework UIKit")
    else()
        list(APPEND _filament_sys_libs "-framework Cocoa" "-framework OpenGL")
    endif()
    target_link_libraries(Filament::filament INTERFACE ${_filament_static_libs} ${_filament_sys_libs})
elseif(WIN32)
    # link.exe resolves archives without a group; opengl32/gdi32/user32 are the WGL platform's,
    # ws2_32 the (unreferenced) matdbg server's.
    target_link_libraries(Filament::filament INTERFACE ${_filament_static_libs}
        opengl32 gdi32 user32 shell32 advapi32 ws2_32)
else()
    if(ANDROID)
        # Bionic folds pthread into libc; Filament drives GLES through libGLESv3.
        set(_filament_sys_libs EGL GLESv3 android log dl)
    else()
        set(_filament_sys_libs EGL GLESv2 dl pthread)
    endif()
    target_link_libraries(Filament::filament INTERFACE
        -Wl,--start-group
        ${_filament_static_libs}
        -Wl,--end-group
        ${_filament_sys_libs}
    )
endif()
