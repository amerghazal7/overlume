# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# ctest build_type_is_set: fails when the library was configured with an empty CMAKE_BUILD_TYPE,
# i.e. compiled with no -O. Guards the Release default in overlume/CMakeLists.txt.
if(NOT BUILD_TYPE)
    message(FATAL_ERROR "CMAKE_BUILD_TYPE is empty: overlume is compiled without optimisation. "
                        "The Release default in overlume/CMakeLists.txt did not apply.")
endif()
message(STATUS "CMAKE_BUILD_TYPE=${BUILD_TYPE}")
