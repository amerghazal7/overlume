# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Install rules. Component "overlume" = shared runtime + headers + assets +
# find_package/pkg-config files; component "static" = the merged archive plus
# every other static archive it needs at link time.

# ponytail: no multiarch libdir; the .so is self-contained
set(CMAKE_INSTALL_LIBDIR lib)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(_ovl_cmake_dir "${CMAKE_INSTALL_LIBDIR}/cmake/overlume")
get_filename_component(_ovl_repo_root "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

if(OVERLUME_BUILD_SHARED)
    install(TARGETS overlume_shared EXPORT overlumeTargets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT overlume
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT overlume
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT overlume
            INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(EXPORT overlumeTargets NAMESPACE overlume:: DESTINATION ${_ovl_cmake_dir}
            COMPONENT overlume)
endif()

install(DIRECTORY include/overlume DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT overlume)
install(DIRECTORY assets/themes/ DESTINATION ${CMAKE_INSTALL_DATADIR}/overlume/themes
        COMPONENT overlume FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY assets/models/ DESTINATION ${CMAKE_INSTALL_DATADIR}/overlume/models
        COMPONENT overlume PATTERN "ATTRIBUTION.md" EXCLUDE)
install(FILES assets/models/ATTRIBUTION.md DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/overlume
        COMPONENT overlume)
install(FILES assets/models/environment/ATTRIBUTION.md
        DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/overlume RENAME ATTRIBUTION-environment.md
        COMPONENT overlume)
foreach(_ovl_doc LICENSE NOTICE)
    if(EXISTS "${_ovl_repo_root}/${_ovl_doc}")
        install(FILES "${_ovl_repo_root}/${_ovl_doc}"
                DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/overlume COMPONENT overlume)
    endif()
endforeach()

configure_package_config_file(cmake/overlumeConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/overlumeConfig.cmake"
    INSTALL_DESTINATION ${_ovl_cmake_dir})
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/overlumeConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
configure_file(cmake/overlume.pc.in "${CMAKE_CURRENT_BINARY_DIR}/overlume.pc" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/overlumeConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/overlumeConfigVersion.cmake"
        DESTINATION ${_ovl_cmake_dir} COMPONENT overlume)
if(NOT WIN32)  # Windows consumers use find_package(overlume); there is no pkg-config there
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/overlume.pc"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/pkgconfig COMPONENT overlume)
endif()
if(WIN32 AND OVERLUME_BUILD_SHARED)
    # App-local VC runtime (the shared library is /MD): vcruntime140*.dll / msvcp140*.dll beside overlume.dll.
    set(CMAKE_INSTALL_UCRT_LIBRARIES FALSE)  # the Universal CRT is an OS component on Windows 10+
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION ${CMAKE_INSTALL_BINDIR})
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT overlume)
    include(InstallRequiredSystemLibraries)
endif()

# ---- static component -------------------------------------------------------
# Walk the static link line (same graph a consumer of in-tree `overlume` gets)
# and collect every archive plus the system libs. libc++/libc++abi/libunwind
# are left out: the consumer's own libc++ provides them.
function(_overlume_collect_static_deps _out_files _out_libs)
    set(_queue ${ARGN})
    set(_seen "")
    set(_files "")
    set(_libs "")
    while(_queue)
        list(POP_FRONT _queue _cur)
        string(REGEX REPLACE "^\\$<(LINK_ONLY|BUILD_INTERFACE):(.*)>$" "\\2" _cur "${_cur}")
        string(REGEX REPLACE "^\\$<(LINK_ONLY|BUILD_INTERFACE):(.*)>$" "\\2" _cur "${_cur}")
        if(_cur MATCHES "^\\$<IF:\\$<TARGET_EXISTS:([^>,]+)>,([^>,]+),([^>,]+)>$")
            if(TARGET "${CMAKE_MATCH_1}")
                set(_cur "${CMAKE_MATCH_2}")
            else()
                set(_cur "${CMAKE_MATCH_3}")
            endif()
        endif()
        # Single-config: resolve $<$<CONFIG:DEBUG>:x> / $<$<NOT:$<CONFIG:DEBUG>>:x> now.
        string(TOUPPER "${CMAKE_BUILD_TYPE}" _bt)
        if(_bt STREQUAL "")
            set(_bt DEBUG)  # no build type: CMake's imported-target config fallback picks debug here
        endif()
        if(_cur MATCHES "^\\$<\\$<CONFIG:([A-Za-z]+)>:(.*)>$")
            string(TOUPPER "${CMAKE_MATCH_1}" _cfg)
            if(_cfg STREQUAL _bt)
                set(_cur "${CMAKE_MATCH_2}")
            else()
                continue()
            endif()
        elseif(_cur MATCHES "^\\$<\\$<BOOL:([^>]*)>:(.*)>$")
            if(CMAKE_MATCH_1)
                set(_cur "${CMAKE_MATCH_2}")
            else()
                continue()
            endif()
        elseif(_cur MATCHES "^\\$<\\$<PLATFORM_ID:([^>]*)>:(.*)>$")
            string(REPLACE "," ";" _plats "${CMAKE_MATCH_1}")
            set(_content "${CMAKE_MATCH_2}")
            if(CMAKE_SYSTEM_NAME IN_LIST _plats)
                set(_cur "${_content}")
            else()
                continue()
            endif()
        elseif(_cur MATCHES "^\\$<\\$<NOT:\\$<CONFIG:([A-Za-z]+)>>:(.*)>$")
            string(TOUPPER "${CMAKE_MATCH_1}" _cfg)
            if(_cfg STREQUAL _bt)
                continue()
            else()
                set(_cur "${CMAKE_MATCH_2}")
            endif()
        endif()
        if(_cur STREQUAL "-framework" AND _queue)
            # "-framework;Foo" arrives as two list items: keep the pair together.
            list(POP_FRONT _queue _fw)
            set(_cur "-framework ${_fw}")
        endif()
        if(_cur STREQUAL "" OR _cur IN_LIST _seen)
            continue()
        endif()
        list(APPEND _seen "${_cur}")
        if(_cur MATCHES "^-Wl,|^-nostdlib|^-stdlib")
            continue()
        elseif(TARGET "${_cur}")
            get_target_property(_type "${_cur}" TYPE)
            if(_type STREQUAL "STATIC_LIBRARY")
                list(APPEND _files "$<TARGET_FILE:${_cur}>")
            elseif(_type STREQUAL "UNKNOWN_LIBRARY")
                # find-module imports (modp_b64, ZLIB, ...): archive or shared object?
                set(_loc "")
                foreach(_p IMPORTED_LOCATION IMPORTED_LOCATION_DEBUG IMPORTED_LOCATION_RELEASE
                           IMPORTED_LOCATION_NOCONFIG)
                    get_target_property(_l "${_cur}" ${_p})
                    if(_l AND NOT _loc)
                        set(_loc "${_l}")
                    endif()
                endforeach()
                if(_loc MATCHES "\\.(a|lib)$")
                    list(APPEND _files "$<TARGET_FILE:${_cur}>")
                elseif(_loc MATCHES "/lib([A-Za-z0-9_+.-]+)\\.(tbd|dylib)$" AND _loc MATCHES "\\.sdk/|^/usr/lib/")
                    list(APPEND _libs "-l${CMAKE_MATCH_1}")  # Apple system library (libz.tbd, ...)
                else()
                    message(FATAL_ERROR "overlume: static package dep '${_cur}' is not an archive: ${_loc}")
                endif()
            elseif(_type STREQUAL "SHARED_LIBRARY")
                message(FATAL_ERROR "overlume: static package dep '${_cur}' is a shared library")
            endif()
            foreach(_prop INTERFACE_LINK_LIBRARIES LINK_LIBRARIES)
                get_target_property(_deps "${_cur}" ${_prop})
                if(_deps AND NOT _type STREQUAL "UTILITY")
                    list(APPEND _queue ${_deps})
                endif()
            endforeach()
        elseif(_cur MATCHES "^-[A-Za-z]")
            list(APPEND _libs "${_cur}")  # -lfoo, -pthread, -framework Foo, ...
        elseif(_cur MATCHES "/([A-Za-z0-9_]+)\\.framework/?$")
            list(APPEND _libs "-framework ${CMAKE_MATCH_1}")
        elseif(_cur MATCHES "/lib([A-Za-z0-9_+.-]+)\\.(tbd|dylib)$" AND _cur MATCHES "\\.sdk/|^/usr/lib/")
            list(APPEND _libs "-l${CMAKE_MATCH_1}")
        elseif(WIN32 AND _cur MATCHES "^[A-Za-z0-9_+.-]+\\.lib$")
            list(APPEND _libs "${_cur}")  # an import library by bare name (advapi32.lib): resolved by link.exe
        elseif(_cur MATCHES "\\.(a|lib)$")
            if(NOT _cur STREQUAL _libcxx_a AND NOT _cur STREQUAL _libcxxabi_a
               AND NOT _cur STREQUAL _libunwind_a)
                list(APPEND _files "${_cur}")
            endif()
        elseif(_cur MATCHES "^\\$<")
            message(FATAL_ERROR "overlume: unhandled generator expression in link graph: [${_cur}]")
        elseif(WIN32 AND _cur MATCHES "^[A-Za-z0-9_+.-]+$")
            list(APPEND _libs "${_cur}")  # a system library by bare name (ws2_32): CMake appends .lib
        elseif(_cur MATCHES "^[A-Za-z0-9_+.-]+$")
            list(APPEND _libs "-l${_cur}")
        else()
            message(FATAL_ERROR "overlume: unhandled link item in static package: ${_cur}")
        endif()
    endwhile()
    list(REMOVE_DUPLICATES _files)
    list(REMOVE_DUPLICATES _libs)
    set(${_out_files} "${_files}" PARENT_SCOPE)
    set(${_out_libs} "${_libs}" PARENT_SCOPE)
endfunction()

_overlume_collect_static_deps(_ovl_dep_files _ovl_dep_libs ${_overlume_private_link})
list(REMOVE_ITEM _ovl_dep_files "$<TARGET_FILE:overlume>")

install(FILES "$<TARGET_FILE:overlume>" DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT static)
set(_ovl_dep_items "")
set(_ovl_idx 0)
set(_ovl_ar_ext a)
set(_ovl_static_name liboverlume.a)
if(WIN32)
    set(_ovl_ar_ext lib)
    set(_ovl_static_name overlume_static.lib)
endif()
foreach(_ovl_dep ${_ovl_dep_files})
    math(EXPR _ovl_idx "${_ovl_idx} + 1")
    if(_ovl_dep MATCHES "^\\$<TARGET_FILE:(.*)>$")
        string(REGEX REPLACE "[^A-Za-z0-9_.+-]" "_" _ovl_base "lib${CMAKE_MATCH_1}.${_ovl_ar_ext}")
    else()
        get_filename_component(_ovl_base "${_ovl_dep}" NAME)
    endif()
    set(_ovl_pad "000${_ovl_idx}")
    string(LENGTH "${_ovl_pad}" _ovl_pad_len)
    math(EXPR _ovl_pad_from "${_ovl_pad_len} - 3")
    string(SUBSTRING "${_ovl_pad}" ${_ovl_pad_from} 3 _ovl_pad3)
    set(_ovl_name "${_ovl_pad3}_${_ovl_base}")
    install(FILES "${_ovl_dep}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/overlume/deps
            RENAME "${_ovl_name}" COMPONENT static)
    list(APPEND _ovl_dep_items "\${_overlume_prefix}/lib/overlume/deps/${_ovl_name}")
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/OverlumeStaticLinkFlags.cmake")
overlume_static_link_flags("${CMAKE_SYSTEM_NAME}" "${OVERLUME_ENABLE_CESIUM}"
    "${_ovl_dep_items}" "${_ovl_dep_libs}"
    _overlume_static_link_libs _overlume_static_link_options)
configure_file(cmake/overlumeStaticTargets.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/overlumeStaticTargets.cmake" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/overlumeStaticTargets.cmake"
        DESTINATION ${_ovl_cmake_dir} COMPONENT static)
