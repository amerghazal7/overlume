# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# cmake -P check of cmake/gen_overlume_def.cmake on canned `dumpbin /symbols` text: overlume:: functions
# are exported; other namespaces, template instantiations of overlume types, undefined symbols and
# constructors are not.
set(_dir "${CMAKE_CURRENT_LIST_DIR}")
set(_work "${CMAKE_CURRENT_BINARY_DIR}/gen_def_test")
file(MAKE_DIRECTORY "${_work}")
file(WRITE "${_work}/sym.txt" [=[
Dump of file x.obj
COFF SYMBOL TABLE
000 00000000 SECT1  notype       Static       | .text
00A 00000000 SECT3  notype ()    External     | ?create_renderer@overlume@@YAPEAVVisualRenderer@1@AEBURenderConfig@1@@Z (class overlume::VisualRenderer * __cdecl overlume::create_renderer(struct overlume::RenderConfig const &))
00B 00000000 SECT4  notype ()    External     | ?make@detail@overlume@@YAHXZ (int __cdecl overlume::detail::make(void))
00C 00000000 SECT5  notype ()    External     | ?load@Node@YAML@@QEAAXXZ (void YAML::Node::load(void))
00D 00000000 SECT6  notype ()    External     | ??$as@M@Node@YAML@@QEBAMXZ
00E 00000000 SECT7  notype ()    External     | ?size@?$vector@Uobject@overlume@@V?$allocator@Uobject@overlume@@@std@@@std@@QEBA_KXZ
00F 00000000 UNDEF  notype ()    External     | ?extern_fn@overlume@@YAXXZ
010 00000000 SECT8  notype ()    External     | ??0Thing@overlume@@QEAA@XZ
011 00000000 SECT9  notype ()    External     | ?make@filament@@YAXXZ
]=])
execute_process(COMMAND ${CMAKE_COMMAND} "-DOUT=${_work}/out.def" "-DSYMBOLS_TEXT=${_work}/sym.txt"
    -P "${_dir}/../../cmake/gen_overlume_def.cmake" RESULT_VARIABLE _rc OUTPUT_VARIABLE _o ERROR_VARIABLE _o)
if(NOT _rc EQUAL 0)
    message(FATAL_ERROR "FAIL: generator errored: ${_o}")
endif()
file(READ "${_work}/out.def" _def)
foreach(_want "?create_renderer@overlume@@" "?make@detail@overlume@@YAHXZ")
    string(FIND "${_def}" "${_want}" _i)
    if(_i EQUAL -1)
        message(FATAL_ERROR "FAIL: ${_want} not exported:\n${_def}")
    endif()
endforeach()
foreach(_bad "YAML" "filament" "vector" "extern_fn" "Thing" "std@@")
    string(FIND "${_def}" "${_bad}" _i)
    if(NOT _i EQUAL -1)
        message(FATAL_ERROR "FAIL: ${_bad} leaked into the export list:\n${_def}")
    endif()
endforeach()
# No overlume symbol at all must be an error, not an empty DLL.
file(WRITE "${_work}/none.txt" "00C 00000000 SECT5  notype ()    External     | ?load@Node@YAML@@QEAAXXZ\n")
execute_process(COMMAND ${CMAKE_COMMAND} "-DOUT=${_work}/none.def" "-DSYMBOLS_TEXT=${_work}/none.txt"
    -P "${_dir}/../../cmake/gen_overlume_def.cmake" RESULT_VARIABLE _rc OUTPUT_QUIET ERROR_QUIET)
if(_rc EQUAL 0)
    message(FATAL_ERROR "FAIL: an object set with no overlume:: symbol was accepted")
endif()
message(STATUS "PASS gen_overlume_def")
