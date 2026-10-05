# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# cmake -P: writes the DLL export list (a .def file) for overlume_shared on Windows.
#   -DOUT=<file.def> and either
#   -DDUMPBIN=<dumpbin.exe> -DOBJLIST=<file, one object path per line>   (the build)
#   -DSYMBOLS_TEXT=<file of "dumpbin /symbols" output>                   (tests/cmake/test_gen_overlume_def.cmake)
# Exports the function symbols defined in namespace overlume and nothing else: the objects also carry
# YAML::/filament::/std:: template instantiations, which must stay private to the DLL.
if(NOT OUT)
    message(FATAL_ERROR "gen_overlume_def: -DOUT is required")
endif()

set(_texts "")
if(SYMBOLS_TEXT)
    file(READ "${SYMBOLS_TEXT}" _t)
    list(APPEND _texts "${_t}")
else()
    file(STRINGS "${OBJLIST}" _objs)
    foreach(_o IN LISTS _objs)
        if(_o STREQUAL "")
            continue()
        endif()
        execute_process(COMMAND "${DUMPBIN}" /nologo /symbols "${_o}"
            OUTPUT_VARIABLE _t RESULT_VARIABLE _rc ERROR_VARIABLE _err)
        if(NOT _rc EQUAL 0)
            message(FATAL_ERROR "gen_overlume_def: dumpbin failed on ${_o}: ${_err}")
        endif()
        list(APPEND _texts "${_t}")
    endforeach()
endif()

set(_names "")
foreach(_t IN LISTS _texts)
    # Defined (SECTn) external functions; undefined ones read "UNDEF".
    string(REGEX MATCHALL "SECT[0-9A-Fa-f]+ +notype( +\\(\\))? +External +\\| +\\?[^ \r\n]+" _hits "${_t}")
    foreach(_h IN LISTS _hits)
        string(REGEX REPLACE "^.*\\| +" "" _n "${_h}")
        if(_n MATCHES "^\\?[A-Za-z0-9_]+(@[A-Za-z0-9_]+)*@overlume@@")
            list(APPEND _names "${_n}")
        endif()
    endforeach()
endforeach()
list(REMOVE_DUPLICATES _names)
list(SORT _names)
list(LENGTH _names _count)
if(_count EQUAL 0)
    message(FATAL_ERROR "gen_overlume_def: no overlume:: function symbols found; refusing to link an empty DLL")
endif()
set(_def "LIBRARY overlume\nEXPORTS\n")
foreach(_n IN LISTS _names)
    string(APPEND _def "    ${_n}\n")
endforeach()
file(WRITE "${OUT}" "${_def}")
message(STATUS "gen_overlume_def: ${_count} exports -> ${OUT}")
