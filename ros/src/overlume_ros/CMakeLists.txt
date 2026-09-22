# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

cmake_minimum_required(VERSION 3.22)
project(overlume_ros LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetVirtualCam.srv)
ament_export_dependencies(rosidl_default_runtime)
find_package(yaml_cpp_vendor REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  yaml_cpp_vendor
  visualization_msgs
  nav_msgs
  map_msgs
  tf2_geometry_msgs
  ament_index_cpp
  diagnostic_msgs
  cv_bridge)

set(OVERLUME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../overlume")
get_filename_component(OVERLUME_DIR "${OVERLUME_DIR}" ABSOLUTE)
set(OVERLUME_LIB "${OVERLUME_DIR}/build/liboverlume.a")
if(NOT EXISTS "${OVERLUME_LIB}")
    message(FATAL_ERROR
        "overlume is not built. Build it first (see "
        "${OVERLUME_DIR}/README.md):\n"
        "  cd ${OVERLUME_DIR}\n"
        "  scripts/setup_toolchain_cesium.sh\n"
        "  cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .\n"
        "  cmake --build build\n"
        "then re-run this configure.")
endif()

file(STRINGS "${OVERLUME_DIR}/cmake/GetFilament.cmake" _filament_version_line
     REGEX "^set\\(FILAMENT_VERSION ")
if(NOT _filament_version_line)
    message(FATAL_ERROR
        "Could not find a FILAMENT_VERSION set() line in "
        "${OVERLUME_DIR}/cmake/GetFilament.cmake")
endif()
string(REGEX REPLACE "^set\\(FILAMENT_VERSION \"([^\"]+)\"\\)$" "\\1"
       FILAMENT_VERSION "${_filament_version_line}")
set(FILAMENT_ROOT "${OVERLUME_DIR}/build/_deps/filament-${FILAMENT_VERSION}/filament"
    CACHE PATH "Filament SDK root (reused from overlume's own build)")
include("${OVERLUME_DIR}/cmake/GetFilament.cmake")

unset(OVERLUME_CLANGXX CACHE)
find_program(OVERLUME_CLANGXX clang++)
if(OVERLUME_CLANGXX)
    execute_process(
        COMMAND "${OVERLUME_CLANGXX}" -stdlib=libc++ -print-file-name=libc++.a
        OUTPUT_VARIABLE _overlume_node_path_libcxx_a
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if(NOT (_overlume_node_path_libcxx_a AND EXISTS "${_overlume_node_path_libcxx_a}"))
        set(OVERLUME_CLANGXX "")
    endif()
endif()
if(NOT OVERLUME_CLANGXX)
    include("${OVERLUME_DIR}/cmake/clang18-toolchain-common.cmake")
    set(OVERLUME_CLANGXX "${_overlume_clang18_wrap_clangxx}")
endif()

execute_process(
    COMMAND "${OVERLUME_CLANGXX}" -stdlib=libc++ -print-file-name=libc++.a
    OUTPUT_VARIABLE _libcxx_a
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_libcxx_a "${_libcxx_a}" REALPATH)
if(NOT EXISTS "${_libcxx_a}")
    message(FATAL_ERROR
        "Could not resolve a static libc++.a from '${OVERLUME_CLANGXX}' "
        "(looked at '${_libcxx_a}').")
endif()
get_filename_component(_libcxx_lib_dir "${_libcxx_a}" DIRECTORY)

set(_overlume_vr_toolchain_cache "${OVERLUME_DIR}/build/CMakeCache.txt")
if(NOT EXISTS "${_overlume_vr_toolchain_cache}")
    message(FATAL_ERROR
        "Cannot verify toolchain match: '${_overlume_vr_toolchain_cache}' not "
        "found (overlume's own build tree has no CMakeCache.txt).")
endif()
file(STRINGS "${_overlume_vr_toolchain_cache}" _overlume_vr_cxx_line
     REGEX "^CMAKE_CXX_COMPILER:(FILEPATH|STRING)=")
if(NOT _overlume_vr_cxx_line)
    message(FATAL_ERROR
        "Cannot verify toolchain match: no CMAKE_CXX_COMPILER entry in "
        "'${_overlume_vr_toolchain_cache}'.")
endif()
string(REGEX REPLACE "^CMAKE_CXX_COMPILER:(FILEPATH|STRING)=" "" _overlume_vr_cxx "${_overlume_vr_cxx_line}")
execute_process(
    COMMAND "${_overlume_vr_cxx}" -stdlib=libc++ -print-file-name=libc++.a
    OUTPUT_VARIABLE _overlume_vr_libcxx_a
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_overlume_vr_libcxx_a "${_overlume_vr_libcxx_a}" REALPATH)
get_filename_component(_overlume_vr_libcxx_lib_dir "${_overlume_vr_libcxx_a}" DIRECTORY)
if(NOT _overlume_vr_libcxx_lib_dir STREQUAL _libcxx_lib_dir)
    message(FATAL_ERROR
        "Toolchain mismatch: this package resolved libc++.a at "
        "'${_libcxx_lib_dir}' (via compiler '${OVERLUME_CLANGXX}'), but "
        "${OVERLUME_LIB} was built with compiler '${_overlume_vr_cxx}' "
        "whose libc++.a lives at '${_overlume_vr_libcxx_lib_dir}'. Linking "
        "these together would mix two different libc++ ABIs into one "
        "binary. Delete this package's build directory and reconfigure "
        "(the find_program() result above may be a stale CACHE entry, or "
        "overlume needs rebuilding with the same toolchain this "
        "package resolves).")
endif()

set(_libcxxabi_a "${_libcxx_lib_dir}/libc++abi.a")
set(_libunwind_a "${_libcxx_lib_dir}/libunwind.a")
foreach(_rt_lib "${_libcxxabi_a}" "${_libunwind_a}")
    if(NOT EXISTS "${_rt_lib}")
        message(FATAL_ERROR
            "Expected static runtime lib not found: ${_rt_lib} (sibling of ${_libcxx_a}).")
    endif()
endforeach()

set(OVERLUME_NODE_STB_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
set(OVERLUME_NODE_STB_DIR "${CMAKE_BINARY_DIR}/_deps/stb")
set(OVERLUME_NODE_STB_TRUETYPE_H "${OVERLUME_NODE_STB_DIR}/stb_truetype.h")
set(OVERLUME_NODE_STB_IMAGE_H "${OVERLUME_NODE_STB_DIR}/stb_image.h")
set(OVERLUME_NODE_STB_IMAGE_WRITE_H "${OVERLUME_NODE_STB_DIR}/stb_image_write.h")
if(NOT EXISTS "${OVERLUME_NODE_STB_TRUETYPE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${OVERLUME_NODE_STB_COMMIT}/stb_truetype.h"
         "${OVERLUME_NODE_STB_TRUETYPE_H}"
         EXPECTED_HASH SHA256=ecd30b05e0dd4fea3a13c26810dd9e1992dc379049482c393d5a19e6b5090aab)
endif()
if(NOT EXISTS "${OVERLUME_NODE_STB_IMAGE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${OVERLUME_NODE_STB_COMMIT}/stb_image.h"
         "${OVERLUME_NODE_STB_IMAGE_H}"
         EXPECTED_HASH SHA256=594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3)
endif()
if(NOT EXISTS "${OVERLUME_NODE_STB_IMAGE_WRITE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${OVERLUME_NODE_STB_COMMIT}/stb_image_write.h"
         "${OVERLUME_NODE_STB_IMAGE_WRITE_H}"
         EXPECTED_HASH SHA256=cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05)
endif()

set(_overlume_node_extra_link_archives "")
set(_overlume_vr_cache "${OVERLUME_DIR}/build/CMakeCache.txt")
if(EXISTS "${_overlume_vr_cache}")
    file(STRINGS "${_overlume_vr_cache}" _overlume_vr_cesium_line
         REGEX "^OVERLUME_ENABLE_CESIUM:BOOL=ON$")
    if(_overlume_vr_cesium_line)
        file(GLOB _overlume_node_cesium_module_archives
             "${OVERLUME_DIR}/build/_deps/cesium-native-build/*/*.a")
        file(GLOB _overlume_node_vcpkg_archives
             "$ENV{HOME}/.ezvcpkg/*/installed/x64-linux-clang-libcxx/lib/*.a"
             "$ENV{HOME}/.ezvcpkg/*/installed/x64-linux-clang-libcxx/debug/lib/*.a")
        set(_overlume_node_extra_link_archives
            ${_overlume_node_cesium_module_archives} ${_overlume_node_vcpkg_archives})
        if(NOT _overlume_node_extra_link_archives)
            message(FATAL_ERROR
                "overlume was built with OVERLUME_ENABLE_CESIUM=ON but "
                "this package found no cesium-native/vcpkg archives to glob "
                "at the expected paths -- the merged liboverlume.a "
                "needs them (sqlite3/curl/openssl/draco/ktx/... symbols) or "
                "this package's own link will fail with undefined "
                "references. Did the paths move at a cesium-native/vcpkg "
                "version bump?")
        endif()
    endif()
endif()

string(JOIN "," _overlume_node_group_items
    "${_libcxx_a}" "${_libcxxabi_a}" "${_libunwind_a}"
    ${_overlume_node_extra_link_archives})
add_library(overlume_prebuilt STATIC IMPORTED)
set_target_properties(overlume_prebuilt PROPERTIES
    IMPORTED_LOCATION "${OVERLUME_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${OVERLUME_DIR}/include"
    INTERFACE_LINK_LIBRARIES "Filament::filament;$<LINK_GROUP:RESCAN,${_overlume_node_group_items}>")

add_library(overlume_node_lib SHARED
  src/overlume_node.cpp
  src/vcam.cpp
  src/tf_adapter.cpp
  src/profile.cpp
  src/frame_transform.cpp
  src/scene_assembly.cpp
  src/adapters/hd_map.cpp
  src/adapters/dynamic_objects.cpp
  src/adapters/path.cpp
  src/adapters/ogm.cpp
  src/adapters/collision.cpp
  src/adapters/generic_marker.cpp
  src/adapters/tf_axes.cpp
  src/adapters/point_cloud.cpp
  src/adapters/trajectory_carpet.cpp
  src/geo_anchor.cpp
  src/diagnostics.cpp
  src/hud_overlay.cpp
  src/callouts.cpp
  src/camera_ingest.cpp
  src/lidar_colorize.cpp
  src/quality_governor.cpp)

target_include_directories(overlume_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(overlume_node_lib PRIVATE "${OVERLUME_NODE_STB_DIR}")
target_include_directories(overlume_node_lib PRIVATE "${OVERLUME_DIR}/src")

ament_target_dependencies(overlume_node_lib ${THIS_PACKAGE_DEPS})
target_link_libraries(overlume_node_lib overlume_prebuilt)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(overlume_node_lib "${cpp_typesupport_target}")

set(_overlume_node_hidden_libs_script "${CMAKE_CURRENT_BINARY_DIR}/overlume_node_hidden_libs.version")
file(WRITE "${_overlume_node_hidden_libs_script}"
"{\n  global:\n    *;\n  local:\n    sqlite3_*;\n    curl_*;\n    SSL_*;\n    EVP_*;\n    inflate*;\n    deflate*;\n    uriParse*;\n    zstd*;\n};\n")
target_link_options(overlume_node_lib PRIVATE
  "LINKER:--version-script=${_overlume_node_hidden_libs_script}")

add_executable(overlume_node src/main.cpp)
target_link_libraries(overlume_node overlume_node_lib)

install(TARGETS overlume_node_lib overlume_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY "${OVERLUME_DIR}/assets/themes"
  DESTINATION share/${PROJECT_NAME}/assets)

install(DIRECTORY assets/fonts
  DESTINATION share/${PROJECT_NAME}/assets)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/assets/ego/M02P.glb"
  DESTINATION share/${PROJECT_NAME}/assets/ego OPTIONAL)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/ego/M02P.glb")
  message(WARNING
    "overlume_ros: assets/ego/M02P.glb not provisioned -- "
    "run scripts/provision_ego_model.sh first (needs the real M02P.obj). "
    "ego_model_path will resolve empty and the node falls back to the clay-box ego.")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ego_anchor test/test_ego_anchor.cpp)
  target_include_directories(test_ego_anchor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)

  ament_add_gtest(test_environment_source_uri test/test_environment_source_uri.cpp)
  target_include_directories(test_environment_source_uri PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_scene_layout test/test_scene_layout.cpp)
  target_include_directories(test_scene_layout PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)

  ament_add_test(check_pod_header
    COMMAND bash "${OVERLUME_DIR}/scripts/check_pod_header.sh"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  add_library(overlume_node_test_paths INTERFACE)
  target_compile_definitions(overlume_node_test_paths INTERFACE
    TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    OVERLUME_THEME_DIR="${OVERLUME_DIR}/assets/themes"
    OVERLUME_NODE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    OVERLUME_NODE_FONT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/NotoSans-Regular.ttf")

  ament_add_gtest(test_profile test/test_profile.cpp src/profile.cpp
    src/adapters/dynamic_objects.cpp src/frame_transform.cpp test/fixture_msgs.cpp)
  target_include_directories(test_profile PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_profile ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_profile ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)
  target_link_libraries(test_profile overlume_prebuilt)

  ament_add_gtest(test_frame_transform test/test_frame_transform.cpp src/frame_transform.cpp)
  target_include_directories(test_frame_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_frame_transform ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_frame_transform ${yaml_cpp_vendor_TARGETS})

  ament_add_gtest(test_scene_assembly test/test_scene_assembly.cpp src/scene_assembly.cpp)
  target_include_directories(test_scene_assembly PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_scene_assembly ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_scene_assembly ${yaml_cpp_vendor_TARGETS})

  ament_add_gtest(test_hd_map_adapter
    test/test_hd_map_adapter.cpp
    src/adapters/hd_map.cpp
    src/adapters/dynamic_objects.cpp
    src/profile.cpp
    src/frame_transform.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_hd_map_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_hd_map_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_hd_map_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_dynamic_objects_adapter
    test/test_dynamic_objects_adapter.cpp
    src/adapters/dynamic_objects.cpp
    src/profile.cpp
    src/frame_transform.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_dynamic_objects_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_dynamic_objects_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_dynamic_objects_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_path_adapter
    test/test_path_adapter.cpp
    src/adapters/path.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_path_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_path_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_path_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_ogm_adapter
    test/test_ogm_adapter.cpp
    src/adapters/ogm.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_ogm_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_ogm_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_ogm_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_collision_adapter
    test/test_collision_adapter.cpp
    src/adapters/collision.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_collision_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_collision_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_collision_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_generic_marker_adapter
    test/test_generic_marker_adapter.cpp
    src/adapters/generic_marker.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_generic_marker_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_generic_marker_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_generic_marker_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_tf_axes_adapter
    test/test_tf_axes_adapter.cpp
    src/adapters/tf_axes.cpp)
  target_include_directories(test_tf_axes_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_tf_axes_adapter ${THIS_PACKAGE_DEPS})

  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp src/diagnostics.cpp)
  target_include_directories(test_diagnostics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_diagnostics diagnostic_msgs)

  ament_add_gtest(test_hud_overlay test/test_hud_overlay.cpp src/hud_overlay.cpp)
  target_include_directories(test_hud_overlay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include
    ${OVERLUME_NODE_STB_DIR})
  ament_target_dependencies(test_hud_overlay ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_hud_overlay overlume_prebuilt overlume_node_test_paths)

  ament_add_gtest(test_callouts test/test_callouts.cpp src/callouts.cpp src/hud_overlay.cpp)
  target_include_directories(test_callouts PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include
    ${OVERLUME_NODE_STB_DIR})
  ament_target_dependencies(test_callouts ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_callouts overlume_prebuilt overlume_node_test_paths)

  ament_add_gtest(test_point_cloud_adapter
    test/test_point_cloud_adapter.cpp
    src/adapters/point_cloud.cpp
    src/frame_transform.cpp
    src/scene_assembly.cpp)
  target_include_directories(test_point_cloud_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include
    ${OVERLUME_NODE_STB_DIR})
  ament_target_dependencies(test_point_cloud_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_point_cloud_adapter overlume_prebuilt overlume_node_test_paths)

  ament_add_gtest(test_trajectory_carpet_adapter
    test/test_trajectory_carpet_adapter.cpp
    src/adapters/trajectory_carpet.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_trajectory_carpet_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_trajectory_carpet_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_trajectory_carpet_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)

  ament_add_gtest(test_geo_anchor test/test_geo_anchor.cpp src/geo_anchor.cpp)
  target_include_directories(test_geo_anchor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_geo_anchor ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_geo_anchor overlume_node_test_paths)

  ament_add_gtest(test_camera_ingest test/test_camera_ingest.cpp src/camera_ingest.cpp)
  target_include_directories(test_camera_ingest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_camera_ingest ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_camera_ingest overlume_prebuilt)

  ament_add_gtest(test_lidar_colorize test/test_lidar_colorize.cpp src/lidar_colorize.cpp)
  target_include_directories(test_lidar_colorize PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include
    ${OVERLUME_DIR}/src)
  target_link_libraries(test_lidar_colorize overlume_prebuilt)

  ament_add_gtest(test_quality_governor test/test_quality_governor.cpp src/quality_governor.cpp)
  target_include_directories(test_quality_governor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_add_test(test_exported_symbols
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/test_exported_symbols.sh"
      "$<TARGET_FILE:overlume_node_lib>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
endif()

ament_package()
