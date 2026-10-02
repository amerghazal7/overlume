# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Link line for the installed static component. GNU ld (Linux, Android) needs the
# dep archives in a group and, with cesium's vendored spdlog/fmt, multiple
# definitions allowed; ld64 and link.exe reject or ignore those flags, and resolve
# the archives without a group.
function(overlume_static_link_flags system cesium dep_items dep_libs out_libs out_opts)
    if(system MATCHES "^(Linux|Android)$")
        set(_libs "-Wl,--start-group;${dep_items};-Wl,--end-group;${dep_libs}")
        set(_opts "")
        if(cesium)
            set(_opts "-Wl,--allow-multiple-definition")
        endif()
    else()
        set(_libs "${dep_items};${dep_libs}")
        set(_opts "")
    endif()
    set(${out_libs} "${_libs}" PARENT_SCOPE)
    set(${out_opts} "${_opts}" PARENT_SCOPE)
endfunction()
