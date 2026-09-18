# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# examples/CMakeLists.txt — builds each examples/*.cpp against overlume's
# PUBLIC API only. add_subdirectory()'d from overlume/CMakeLists.txt (under
# the OVERLUME_BUILD_EXAMPLES option), so this file never calls project() of
# its own -- PROJECT_SOURCE_DIR/STB_DIR/the `overlume`/`overlume::overlume`
# targets below are all inherited from that same overlume project scope,
# already defined above the add_subdirectory() call.
file(GLOB _overlume_examples_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
foreach(_example_src ${_overlume_examples_srcs})
    get_filename_component(_example_name "${_example_src}" NAME_WE)
    add_executable(${_example_name} "${_example_src}")
    # STB_DIR: stb_image_write.h, downloaded once by overlume/CMakeLists.txt,
    # for common.hpp's PNG writer. CMAKE_CURRENT_SOURCE_DIR: so every example
    # can #include "common.hpp".
    target_include_directories(${_example_name} PRIVATE "${STB_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    # PUBLIC-API-only consumer: link the alias target the way an external
    # consumer would, never overlume's internal src/ headers.
    target_link_libraries(${_example_name} PRIVATE overlume::overlume)
    # OVERLUME_EXAMPLES_THEME_DIR: the shipped theme dir, same OVERLUME_..._DIR
    # baked-in-compile-definition pattern as the test suite's
    # OVERLUME_DEFAULT_THEME_DIR (see overlume/tests/test_paths.hpp).
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_THEME_DIR="${PROJECT_SOURCE_DIR}/assets/themes")
    # Only 05_environment.cpp needs the baked-chunk fixture dir; defining it
    # for every target is harmless (an unused #define elsewhere) and keeps
    # this loop uniform.
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_ENV_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/tests/fixtures/environment_test_town_0")
    # OVERLUME_EXAMPLES_MODEL_DIR: the shipped per-class clay-model dir
    # (VM-022 / set_object_model_dir()), same OVERLUME_..._DIR
    # baked-in-compile-definition pattern as OVERLUME_EXAMPLES_THEME_DIR above.
    target_compile_definitions(${_example_name} PRIVATE
        OVERLUME_EXAMPLES_MODEL_DIR="${PROJECT_SOURCE_DIR}/assets/models")
endforeach()
