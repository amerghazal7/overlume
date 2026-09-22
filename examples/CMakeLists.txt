# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

file(GLOB _overlume_examples_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
foreach(_example_src ${_overlume_examples_srcs})
    get_filename_component(_example_name "${_example_src}" NAME_WE)
    add_executable(${_example_name} "${_example_src}")
    target_include_directories(${_example_name} PRIVATE "${STB_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${_example_name} PRIVATE overlume::overlume)
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_THEME_DIR="${PROJECT_SOURCE_DIR}/assets/themes")
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_ENV_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/tests/fixtures/environment_test_town_0")
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_MODEL_DIR="${PROJECT_SOURCE_DIR}/assets/models")
endforeach()
