cmake_minimum_required(VERSION 3.22)
project(micropilot_visualization_node LANGUAGES C CXX)

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
# SetVirtualCam.srv (plan Task 5 / spec §6) — existing srv package, a
# dependency not a copy; micropilot_rendering_node is a
# rosidl_interface_packages member so find_package resolves its generated
# C++ headers (micropilot_rendering_node/srv/set_virtual_cam.hpp).
find_package(micropilot_rendering_node REQUIRED)
# Epic 2 Task 1 (VM-020): profile loader (yaml_cpp_vendor -- NOT
# find_package(yaml-cpp) directly, see the long comment on _yamlcpp_a below
# for why that distinction matters) + the message types the profile's
# `type:` rows and every later adapter need.
find_package(yaml_cpp_vendor REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# profile_dir: "" -> this package's own installed share/config (on_configure).
find_package(ament_index_cpp REQUIRED)
# Epic 3 Task 2 (VM-034): ~/diagnostics message type -- ROS2 Humble core
# package, already present in this distro.
find_package(diagnostic_msgs REQUIRED)
# VM-091 (unified-engine migration Task 2 Step 6): camera_ingest.cpp's
# cv_bridge::toCvCopy(msg, "rgb8") -- the same dependency
# micropilot_rendering_node/CMakeLists.txt already carries for the
# identical call.
find_package(cv_bridge REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  micropilot_rendering_node
  yaml_cpp_vendor
  visualization_msgs
  nav_msgs
  map_msgs
  tf2_geometry_msgs
  ament_index_cpp
  diagnostic_msgs
  cv_bridge)

# ── visual_renderer (clang/libc++ static lib) — cross-toolchain import ──────
# visual_renderer (docs/superpowers/plans/2026-08-18-visual-mode.md Epic 0
# Task 1/2) builds standalone with clang/libc++ (see its README.md) and
# cannot be add_subdirectory'd here: this package builds stock gcc/libstdc++
# (Global Constraints / spec §2 — mixing the two runtimes at a std:: boundary
# is UB). It also has no install()/find_package config of its own (this node
# is its first consumer), so this package imports the already-built static
# archive directly instead of via find_package.
#
# Why this actually links: an Itanium-ABI static archive links into a gcc
# binary regardless of which compiler produced it (same ABI, same target
# triple) as long as nothing std:: crosses the boundary — which is exactly
# what api.h's POD-only contract guarantees. libc++'s `std::__1` inline
# namespace keeps its mangled symbols distinct from this package's
# libstdc++ ones, so no duplicate-symbol clash at final link time.
set(VISUAL_RENDERER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/visual_renderer")
get_filename_component(VISUAL_RENDERER_DIR "${VISUAL_RENDERER_DIR}" ABSOLUTE)
set(VISUAL_RENDERER_LIB "${VISUAL_RENDERER_DIR}/build/libvisual_renderer.a")
if(NOT EXISTS "${VISUAL_RENDERER_LIB}")
    message(FATAL_ERROR
        "visual_renderer is not built. Build it first (see "
        "${VISUAL_RENDERER_DIR}/README.md):\n"
        "  cd ${VISUAL_RENDERER_DIR}\n"
        "  scripts/setup_toolchain.sh\n"
        "  cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .\n"
        "  cmake --build build\n"
        "then re-run this configure.")
endif()

# Reuse visual_renderer's own cmake/GetFilament.cmake (unmodified — just
# include()'d) instead of re-declaring its ~30-archive link recipe here.
# Pre-seed FILAMENT_ROOT to the SDK that lib's own build already fetched and
# sha256-verified, so this second configure doesn't re-download ~50 MB: the
# script's own `set(FILAMENT_ROOT ... CACHE PATH ...)` is a no-op once the
# variable is already in the cache, and it skips the fetch when
# FILAMENT_ROOT/include/filament/Engine.h already exists.
# FILAMENT_VERSION single-sourced from visual_renderer's own
# cmake/GetFilament.cmake (Step (g), VM-037) -- this used to be a second
# hardcoded "1.56.5" literal here, which would silently go stale after a pin
# bump there (this node would keep linking the stale SDK with no error).
# Extracted via file(STRINGS) rather than include()'d directly: FILAMENT_ROOT
# below needs the version string BEFORE GetFilament.cmake's own
# set(FILAMENT_ROOT ...) runs (its CACHE var would otherwise win first and
# never get re-pointed at this line's composed path).
file(STRINGS "${VISUAL_RENDERER_DIR}/cmake/GetFilament.cmake" _filament_version_line
     REGEX "^set\\(FILAMENT_VERSION ")
if(NOT _filament_version_line)
    message(FATAL_ERROR
        "Could not find a FILAMENT_VERSION set() line in "
        "${VISUAL_RENDERER_DIR}/cmake/GetFilament.cmake")
endif()
string(REGEX REPLACE "^set\\(FILAMENT_VERSION \"([^\"]+)\"\\)$" "\\1"
       FILAMENT_VERSION "${_filament_version_line}")
set(FILAMENT_ROOT "${VISUAL_RENDERER_DIR}/build/_deps/filament-${FILAMENT_VERSION}/filament"
    CACHE PATH "Filament SDK root (reused from visual_renderer's own build)")
include("${VISUAL_RENDERER_DIR}/cmake/GetFilament.cmake")

# Locate the same rootless clang toolchain visual_renderer's own
# scripts/setup_toolchain.sh bootstraps, purely as a TOOL to resolve the
# static libc++/libc++abi/libunwind archive paths below — this package's own
# sources still compile with gcc throughout; nothing here recompiles with
# clang. See visual_renderer/CMakeLists.txt for why sibling-of-libc++.a (not
# a bare -lunwind) is required: this box's system libunwind package is an
# unrelated, ABI-incompatible library of the same name.
find_program(MPVIZ_CLANGXX clang++
    PATHS "$ENV{XDG_CACHE_HOME}/mpviz-toolchain/bin" "$ENV{HOME}/.cache/mpviz-toolchain/bin"
    NO_DEFAULT_PATH)
if(NOT MPVIZ_CLANGXX)
    find_program(MPVIZ_CLANGXX clang++)
endif()
if(NOT MPVIZ_CLANGXX)
    message(FATAL_ERROR
        "No clang++ found (checked visual_renderer's rootless toolchain "
        "prefix and PATH). Run ${VISUAL_RENDERER_DIR}/scripts/setup_toolchain.sh "
        "first (it is idempotent).")
endif()

execute_process(
    COMMAND "${MPVIZ_CLANGXX}" -stdlib=libc++ -print-file-name=libc++.a
    OUTPUT_VARIABLE _libcxx_a
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_libcxx_a "${_libcxx_a}" REALPATH)
if(NOT EXISTS "${_libcxx_a}")
    message(FATAL_ERROR
        "Could not resolve a static libc++.a from '${MPVIZ_CLANGXX}' "
        "(looked at '${_libcxx_a}').")
endif()
get_filename_component(_libcxx_lib_dir "${_libcxx_a}" DIRECTORY)
set(_libcxxabi_a "${_libcxx_lib_dir}/libc++abi.a")
set(_libunwind_a "${_libcxx_lib_dir}/libunwind.a")
foreach(_rt_lib "${_libcxxabi_a}" "${_libunwind_a}")
    if(NOT EXISTS "${_rt_lib}")
        message(FATAL_ERROR
            "Expected static runtime lib not found: ${_rt_lib} (sibling of ${_libcxx_a}).")
    endif()
endforeach()

# yaml-cpp (Epic 1 Task 2 / VM-011) -- WAS resolved here as a second,
# clang/libc++ static archive (${VISUAL_RENDERER_DIR}/build/_deps/
# yamlcpp-build/libyaml-cpp.a) so the node's link could satisfy
# libvisual_renderer.a's undefined YAML::* symbols. Epic 2 Task 1 (VM-020)
# Step 0.2 removed the need for that: visual_renderer's own CMakeLists.txt
# now `ld -r`s that archive's objects into libvisual_renderer.a and RENAMES
# every YAML::-namespaced symbol (see that file's POST_BUILD comment and
# scripts/merge_yamlcpp.sh for the full history -- two earlier variants of
# this fix, localize-by-visibility and localize-by-name, both failed for
# concretely different reasons, one of them ONLY once real node-side yaml
# usage existed to expose it), so the members this node used to need to
# supply now live INSIDE libvisual_renderer.a under renamed symbol names --
# nothing here resolves the ORIGINAL YAML:: names anymore, and nothing here
# may reference that clang/libc++ archive's headers or symbols directly
# (spec §2: mixing libc++ and libstdc++ at a std:: boundary is UB). This
# node's OWN yaml-cpp need (profile.cpp, fixture_msgs.cpp) is
# `yaml_cpp_vendor` above -- the distro's gcc/libstdc++ copy, a completely
# separate archive from the one that used to live here. Do NOT
# "helpfully" re-add a libyaml-cpp.a reference here: that reintroduces the
# exact two-copies-of-the-same-symbol-name hazard this rename exists to
# rule out (test_profile.cpp's CoexistsWithTheRendererLibrarysOwnYamlCpp
# is the regression test for it, and it needs a CORRECT row count out of
# load_profile(), not just "doesn't crash" -- a prior, since-abandoned fix
# attempt linked fine and still returned the wrong data).

# ── stb_truetype.h / stb_image.h / stb_image_write.h (Epic 3 Task 3 / VM-030,
# Step 1) — a SEPARATE vendoring from visual_renderer's own (that library's
# file(DOWNLOAD ...) block, CMakeLists.txt:95-114, lands in ITS OWN
# CMAKE_BINARY_DIR/_deps/stb; this package never add_subdirectory's that
# project and only imports its prebuilt archive below, so nothing from that
# build tree is reachable here — and that archive never vendored
# stb_truetype.h in the first place (stb_image_write.h doesn't rasterize
# glyphs). Same immutable-commit-URL + EXPECTED_HASH pattern, into this
# package's OWN build tree. stb_image.h/stb_image_write.h are pulled too,
# for the node-side HUD golden test only (test_hud_overlay.cpp: PNG-encode
# the actual frame, PNG-decode a promoted golden to SSIM against) — same
# commit as visual_renderer's own pin, so the two vendored copies are
# byte-identical, just living in different build trees.
set(MPVIZ_NODE_STB_COMMIT "2c980bb59875b0d32144a71867fbdebb2f77cd20")
set(MPVIZ_NODE_STB_DIR "${CMAKE_BINARY_DIR}/_deps/stb")
set(MPVIZ_NODE_STB_TRUETYPE_H "${MPVIZ_NODE_STB_DIR}/stb_truetype.h")
set(MPVIZ_NODE_STB_IMAGE_H "${MPVIZ_NODE_STB_DIR}/stb_image.h")
set(MPVIZ_NODE_STB_IMAGE_WRITE_H "${MPVIZ_NODE_STB_DIR}/stb_image_write.h")
if(NOT EXISTS "${MPVIZ_NODE_STB_TRUETYPE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${MPVIZ_NODE_STB_COMMIT}/stb_truetype.h"
         "${MPVIZ_NODE_STB_TRUETYPE_H}"
         EXPECTED_HASH SHA256=ecd30b05e0dd4fea3a13c26810dd9e1992dc379049482c393d5a19e6b5090aab)
endif()
if(NOT EXISTS "${MPVIZ_NODE_STB_IMAGE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${MPVIZ_NODE_STB_COMMIT}/stb_image.h"
         "${MPVIZ_NODE_STB_IMAGE_H}"
         EXPECTED_HASH SHA256=594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3)
endif()
if(NOT EXISTS "${MPVIZ_NODE_STB_IMAGE_WRITE_H}")
    file(DOWNLOAD "https://raw.githubusercontent.com/nothings/stb/${MPVIZ_NODE_STB_COMMIT}/stb_image_write.h"
         "${MPVIZ_NODE_STB_IMAGE_WRITE_H}"
         EXPECTED_HASH SHA256=cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05)
endif()

add_library(visual_renderer_prebuilt STATIC IMPORTED)
set_target_properties(visual_renderer_prebuilt PROPERTIES
    IMPORTED_LOCATION "${VISUAL_RENDERER_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${VISUAL_RENDERER_DIR}/include"
    INTERFACE_LINK_LIBRARIES "Filament::filament;${_libcxx_a};${_libcxxabi_a};${_libunwind_a}")

# ── Node library ─────────────────────────────────────────────────────────────
# Hand-written list, NOT a glob (CONFIGURE_DEPENDS globs are used for the
# library's tests, not here) -- a source added to this package and left off
# this list is a silent no-op, not a build error.
add_library(visualization_node_lib SHARED
  src/visualization_node.cpp
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
  src/camera_ingest.cpp)

target_include_directories(visualization_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# hud_overlay.cpp's #define STB_TRUETYPE_IMPLEMENTATION -- see that file --
# needs stb_truetype.h on the include path; PRIVATE, not PUBLIC (the header
# stays an implementation detail of this target, not part of this
# library's own public include/ surface).
target_include_directories(visualization_node_lib PRIVATE "${MPVIZ_NODE_STB_DIR}")

ament_target_dependencies(visualization_node_lib ${THIS_PACKAGE_DEPS})
target_link_libraries(visualization_node_lib visual_renderer_prebuilt)

# ── Executable ───────────────────────────────────────────────────────────────
add_executable(visualization_node src/main.cpp)
target_link_libraries(visualization_node visualization_node_lib)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS visualization_node_lib visualization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# VM-044: package the theme YAMLs (plain-git-committed in visual_renderer,
# same convention this line follows) into THIS package's own share dir, so
# on_configure()'s ament_index_cpp resolution (theme_assets_dir/
# initial_theme, both default "") finds a real install instead of silently
# landing on DEFAULT_THEME_ASSETS_DIR's compiled-in path to this checkout's
# source tree.
install(DIRECTORY "${VISUAL_RENDERER_DIR}/assets/themes"
  DESTINATION share/${PROJECT_NAME}/assets)

# HUD compositor font (VM-030/VM-044): same ament_index resolution as the
# themes above -- installed so on_configure()'s hud_font_path fallback finds
# a real share/ path instead of a per-checkout one.
install(DIRECTORY assets/fonts
  DESTINATION share/${PROJECT_NAME}/assets)

# VM-044: the converted M02P ego glTF. NOT committed to git like the themes/
# fonts above -- the source OBJ is ~143 MB and the converted glb ~72 MB,
# past the plain-git convention's practical size (the backlog's own
# documented alternative for an "enormous" binary asset: a fetch/provision
# script instead of Git LFS, since this repo has no LFS remote configured).
# scripts/provision_ego_model.sh converts it from the real M02P.obj into
# assets/ego/M02P.glb (gitignored); install it only if that has actually
# been run -- a missing file here is a real, honestly-recorded gap
# (ego_model_path resolves "" and the node's existing clay-box fallback
# takes over), not a build error.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/ego/M02P.glb")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/assets/ego/M02P.glb"
    DESTINATION share/${PROJECT_NAME}/assets/ego)
else()
  message(WARNING
    "micropilot_visualization_node: assets/ego/M02P.glb not provisioned -- "
    "run scripts/provision_ego_model.sh first (needs the real M02P.obj). "
    "ego_model_path will resolve empty and the node falls back to the clay-box ego.")
endif()

# ── Tests ────────────────────────────────────────────────────────────────────
# compose_ego_anchored_pose/kEgoForwardYaw (ego_anchor.hpp) unit coverage
# (review finding: the yaw convention already regressed once with zero
# automated coverage). Header-only + POD types only -- no need to link the
# visual_renderer static archive, just its headers.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ego_anchor test/test_ego_anchor.cpp)
  target_include_directories(test_ego_anchor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)

  # Epic 3 Task 1 (VM-036) / this epic's slice of VM-037(e): mirrors
  # tests/test_scene_buffer.cpp's sizeof/offsetof static_asserts, compiled
  # gcc/libstdc++, so a future scene.h layout drift fails the NODE build too,
  # not only the library's own clang/libc++ test binary (ADR-0004). Header-only
  # + POD types only -- no need to link the visual_renderer static archive,
  # just its headers, same pattern as test_ego_anchor immediately above. The
  # static_asserts do the real work; the TEST body is a placeholder so this is
  # a normal gtest target ctest/colcon can run.
  ament_add_gtest(test_scene_layout test/test_scene_layout.cpp)
  target_include_directories(test_scene_layout PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)

  # Step (f), VM-037: visual_renderer's own POD-boundary check
  # (scripts/check_pod_header.sh, add_test()'d in its OWN standalone
  # clang/libc++ CMake project) was reachable only from that project's own
  # ctest -- colcon_build.sh never builds or runs it, so a std:: leak into
  # scene.h/api.h shipped undetected through every `colcon test` run.
  # Wiring it in here (this package already depends on VISUAL_RENDERER_DIR
  # above) closes that gap for the build everyone actually runs.
  # GENERATE_RESULT_FOR_RETURN_CODE_ZERO: check_pod_header.sh is a plain bash
  # script, not a gtest binary -- it produces no xunit result file of its own,
  # so ament_add_test's run_test.py wrapper needs telling that exit code 0
  # means pass (its default assumes the command writes its own result file).
  ament_add_test(check_pod_header
    COMMAND bash "${VISUAL_RENDERER_DIR}/scripts/check_pod_header.sh"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO)

  # Epic 2 Task 1 (VM-020) Step 1 item 6: every gtest target that compiles
  # test/fixture_msgs.cpp needs these three -- fixture_msgs.cpp's
  # urban_row()/sim_row() use TEST_CONFIG_DIR unconditionally, and
  # load_marker_array()-style relative fixture paths resolve against
  # MPVIZ_NODE_FIXTURES_DIR. Factored ONCE as an INTERFACE target instead of
  # copy-pasting three target_compile_definitions calls per adapter gtest
  # (Tasks 2-8 add more of those; this is the one place to look).
  add_library(mpviz_node_test_paths INTERFACE)
  target_compile_definitions(mpviz_node_test_paths INTERFACE
    TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    MPVIZ_THEME_DIR="${VISUAL_RENDERER_DIR}/assets/themes"
    MPVIZ_NODE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
    # Epic 3 Task 3 (VM-030): the one committed HUD font, for
    # test_hud_overlay's CompositeHud() calls -- same "test macro, not a
    # hardcoded relative path" shape as TEST_CONFIG_DIR/MPVIZ_THEME_DIR above.
    MPVIZ_NODE_FONT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/NotoSans-Regular.ttf")

  ament_add_gtest(test_profile test/test_profile.cpp src/profile.cpp
    src/adapters/dynamic_objects.cpp src/frame_transform.cpp test/fixture_msgs.cpp)
  target_include_directories(test_profile PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_profile ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_profile ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)
  # The ONE node gtest that links visual_renderer_prebuilt -- Step 0.3's
  # coexistence test calls mpviz::theme_parses, a symbol in
  # libvisual_renderer.a, which drags in Filament and static libc++.
  # test_ego_anchor deliberately links headers only (above) precisely to
  # avoid that; this target is the documented exception.
  target_link_libraries(test_profile visual_renderer_prebuilt)

  ament_add_gtest(test_frame_transform test/test_frame_transform.cpp src/frame_transform.cpp)
  target_include_directories(test_frame_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_frame_transform ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_frame_transform ${yaml_cpp_vendor_TARGETS})

  ament_add_gtest(test_scene_assembly test/test_scene_assembly.cpp src/scene_assembly.cpp)
  target_include_directories(test_scene_assembly PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_scene_assembly ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_scene_assembly ${yaml_cpp_vendor_TARGETS})

  # Epic 2 Task 2 (VM-020): HdMapAdapter. Needs profile.cpp (ProfileRow/
  # classify/load_profile), frame_transform.cpp (FrameTransformer) and
  # fixture_msgs.cpp (load_marker_array/urban_row/sim_row) -- same
  # mpviz_node_test_paths INTERFACE target every fixture-backed adapter
  # gtest uses. No visual_renderer_prebuilt link: scene.h's MapElement/Vec3
  # are POD-only, header visibility is enough (test_scene_assembly's own
  # precedent).
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_hd_map_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_hd_map_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 3 (VM-021): DynamicObjectsAdapter + class inference. Same
  # shape as test_hd_map_adapter above -- no visual_renderer_prebuilt link,
  # scene.h's TrackedObject/Vec3 are POD-only.
  ament_add_gtest(test_dynamic_objects_adapter
    test/test_dynamic_objects_adapter.cpp
    src/adapters/dynamic_objects.cpp
    src/profile.cpp
    src/frame_transform.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_dynamic_objects_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_dynamic_objects_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_dynamic_objects_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 5 (VM-023): PathAdapter. Same shape as test_hd_map_adapter/
  # test_dynamic_objects_adapter above -- no visual_renderer_prebuilt link,
  # scene.h's PathRibbon/Vec3 are POD-only.
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_path_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_path_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 6 (VM-025): OgmAdapter. Same shape as test_hd_map_adapter/
  # test_dynamic_objects_adapter/test_path_adapter above -- no
  # visual_renderer_prebuilt link, scene.h's GroundGridLayer/Vec3 are
  # POD-only.
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_ogm_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_ogm_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 7 (VM-026): CollisionAdapter. Same shape as
  # test_hd_map_adapter/test_dynamic_objects_adapter/test_path_adapter/
  # test_ogm_adapter above -- no visual_renderer_prebuilt link, scene.h's
  # AlertPolygon/Vec3 are POD-only.
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_collision_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_collision_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 8 (VM-027): GenericMarkerAdapter, the spec §7 parity
  # guarantee. Same shape as test_hd_map_adapter/test_dynamic_objects_
  # adapter/test_path_adapter/test_ogm_adapter/test_collision_adapter
  # above -- no visual_renderer_prebuilt link, scene.h's GenericMarker/Vec3
  # are POD-only.
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_generic_marker_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_generic_marker_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # Epic 2 Task 8 Step 7 (VM-027): TfAxesAdapter, the TF-axes debug-layer
  # PRODUCER. No FrameTransformer/profile.cpp/fixture_msgs.cpp dependency
  # (this file's own header comment: it takes the raw tf2_ros::Buffer
  # directly, and its row is hand-built in the test rather than loaded
  # from a shipped profile -- `adapter: tf_axes` rows carry no topic, so
  # urban_row()/sim_row()'s by-topic lookup can't name one anyway).
  ament_add_gtest(test_tf_axes_adapter
    test/test_tf_axes_adapter.cpp
    src/adapters/tf_axes.cpp)
  target_include_directories(test_tf_axes_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_tf_axes_adapter ${THIS_PACKAGE_DEPS})

  # Epic 3 Task 2 (VM-034) Step 0: BuildDiagnostics() is a pure data
  # transform (diagnostics.hpp's own header comment) -- no fixture_msgs, no
  # profile.cpp, no visual_renderer_prebuilt link needed, same "header/POD
  # only" shape as test_ego_anchor/test_scene_layout above.
  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp src/diagnostics.cpp)
  target_include_directories(test_diagnostics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_diagnostics diagnostic_msgs)

  # Epic 3 Task 3 (VM-030): CompositeHud()/PopulateHud() + the sanctioned-red
  # node-side 720p HUD golden. The ONE OTHER node gtest (besides test_profile)
  # that links visual_renderer_prebuilt -- the golden case stands up a real
  # headless-EGL VisualRenderer to get a realistic frame to composite onto
  # (test_profile's own comment explains why that link is the documented
  # exception, not the rule). Needs the node's own stb_truetype/_image/
  # _image_write vendoring (Step 1), not visual_renderer's -- see this
  # file's own comment on MPVIZ_NODE_STB_DIR above for why that split exists.
  ament_add_gtest(test_hud_overlay test/test_hud_overlay.cpp src/hud_overlay.cpp)
  target_include_directories(test_hud_overlay PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include
    ${MPVIZ_NODE_STB_DIR})
  ament_target_dependencies(test_hud_overlay ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_hud_overlay visual_renderer_prebuilt mpviz_node_test_paths)

  # Epic 3 Task 4 (VM-031): BuildNearestCallout()'s camera-tracking / behind-
  # camera-suppression behavior. Needs a real headless-EGL VisualRenderer
  # (project_to_screen() reads the live r->camera state render_frame() just
  # set) -- links visual_renderer_prebuilt, same documented-exception shape
  # test_profile/test_hud_overlay already use.
  ament_add_gtest(test_callouts test/test_callouts.cpp src/callouts.cpp src/hud_overlay.cpp)
  target_include_directories(test_callouts PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include
    ${MPVIZ_NODE_STB_DIR})
  ament_target_dependencies(test_callouts ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_callouts visual_renderer_prebuilt mpviz_node_test_paths)

  # Epic 3 Task 6 (VM-035): PointCloudAdapter + the three sanctioned-red
  # synthetic-scene point-cloud goldens (one per auto tier). Needs
  # visual_renderer_prebuilt (the goldens stand up a real headless-EGL
  # VisualRenderer, same documented-exception shape test_profile/
  # test_hud_overlay/test_callouts already use) and the node's own
  # stb_truetype/_image/_image_write vendoring (test_hud_overlay's own
  # comment explains the split from visual_renderer's copy). No
  # profile.cpp/fixture_msgs.cpp link -- this adapter's tests hand-build
  # their own ProfileRow/PointCloud2, same "no shipped row to borrow" shape
  # test_tf_axes_adapter uses (FIXTURE GAP: no live point_cloud row exists
  # in any shipped profile).
  ament_add_gtest(test_point_cloud_adapter
    test/test_point_cloud_adapter.cpp
    src/adapters/point_cloud.cpp
    src/frame_transform.cpp
    src/scene_assembly.cpp)
  target_include_directories(test_point_cloud_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include
    ${MPVIZ_NODE_STB_DIR})
  ament_target_dependencies(test_point_cloud_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_point_cloud_adapter visual_renderer_prebuilt mpviz_node_test_paths)

  # VM-077: TrajectoryCarpetAdapter. Same shape as test_collision_adapter/
  # test_generic_marker_adapter above -- no visual_renderer_prebuilt link,
  # scene.h's TrajectoryCarpet/PointCloudPoint/Vec3 are POD-only. No shipped
  # profile row to borrow via urban_row() for the ingest tests themselves
  # (the row lands in this same task, Step 3) -- hand-built ProfileRow, same
  # "no urban_row()/sim_row() to borrow" shape test_point_cloud_adapter.cpp
  # already uses for its own row.
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
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_trajectory_carpet_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_trajectory_carpet_adapter ${yaml_cpp_vendor_TARGETS} mpviz_node_test_paths)

  # VM-050 (Epic 4 Task 1): GeoAnchor pure math + GeoAnchorSolver. Hand-
  # written-list trap (Global Constraints): src/geo_anchor.cpp is appended
  # to visualization_node_lib's sources above AND here -- both, or this
  # target silently links against a stale copy. No visual_renderer_prebuilt
  # link needed (GeoAnchor/Vec3 are POD-only, same as test_ego_anchor/
  # test_scene_layout above); needs MPVIZ_NODE_FIXTURES_DIR for the
  # committed geo_anchor_samples_0.csv fixture (Step 1).
  ament_add_gtest(test_geo_anchor test/test_geo_anchor.cpp src/geo_anchor.cpp)
  target_include_directories(test_geo_anchor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_geo_anchor ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_geo_anchor mpviz_node_test_paths)

  # VM-091 (unified-engine migration Task 2 Step 6): camera_ingest.hpp's
  # pure math (twist_at/rig_delta/compensation_delta_4x4/
  # OrthonormalizeExtrinsics) + IngestState's info_ready gate/monotonic
  # frame_id bookkeeping. Header/POD only, same "no visual_renderer_prebuilt
  # link needed" shape as test_ego_anchor/test_scene_layout/test_geo_anchor
  # above -- scene.h's CameraExtrinsics/CameraIntrinsics are POD. No
  # rclcpp-node harness (this suite has none; see this test file's own
  # header comment) -- links cv_bridge only because camera_ingest.cpp
  # (compiled into this same gtest binary) includes cv_bridge/cv_bridge.h.
  ament_add_gtest(test_camera_ingest test/test_camera_ingest.cpp src/camera_ingest.cpp)
  target_include_directories(test_camera_ingest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VISUAL_RENDERER_DIR}/include)
  ament_target_dependencies(test_camera_ingest ${THIS_PACKAGE_DEPS})
  # camera_ingest.cpp calls mpviz::set_camera_frame/set_camera_motion_delta
  # (defined in libvisual_renderer.a, not header-only) even though this
  # test's own cases never trigger those call sites -- the whole TU still
  # needs the symbols to link. Same documented-exception shape as
  # test_profile/test_hud_overlay/test_callouts above.
  target_link_libraries(test_camera_ingest visual_renderer_prebuilt)
endif()

ament_package()
