# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# cmake -P check: the installed static config carries GNU-ld-only flags on
# Linux/Android and none elsewhere (ld64 rejects --start-group).
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/OverlumeStaticLinkFlags.cmake")

function(expect system cesium want_group want_multi)
    overlume_static_link_flags("${system}" "${cesium}" "a.a;b.a" "pthread" libs opts)
    string(FIND "${libs}" "--start-group" g)
    # GNU ld spells it --allow-multiple-definition, link.exe /FORCE:MULTIPLE.
    string(FIND "${opts}" "--allow-multiple-definition" m)
    if(m EQUAL -1)
        string(FIND "${opts}" "/FORCE:MULTIPLE" m)
    endif()
    if((g GREATER -1) AND NOT want_group OR (g EQUAL -1) AND want_group)
        message(FATAL_ERROR "${system}: group flags wrong: '${libs}'")
    endif()
    if((m GREATER -1) AND NOT want_multi OR (m EQUAL -1) AND want_multi)
        message(FATAL_ERROR "${system}: multiple-definition flag wrong: '${opts}'")
    endif()
    if(NOT libs MATCHES "a\\.a;b\\.a" OR NOT libs MATCHES "pthread")
        message(FATAL_ERROR "${system}: dep archives/libs lost: '${libs}'")
    endif()
endfunction()

expect(Linux ON TRUE TRUE)
expect(Linux OFF TRUE FALSE)
expect(Android ON TRUE TRUE)
expect(Darwin ON FALSE FALSE)
expect(iOS ON FALSE FALSE)
expect(Windows ON FALSE TRUE)
message(STATUS "PASS static link flags per platform")
