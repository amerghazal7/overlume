cmake_minimum_required(VERSION 3.18)

# visual_renderer is its own CMake project (not add_subdirectory'd into the
# gcc-toolchain `cuda/` build): the toolchain boundary in the design spec
# (docs/superpowers/specs/2026-08-18-visual-mode-design.md §2) requires this
# library to compile clang/libc++ while the rest of the CUDA/ROS stack stays
# on gcc/libstdc++ — CMAKE_CXX_COMPILER is fixed per top-level project, so the
# split has to happen at this granularity. Consumers (the ROS node) link
# against it only through the POD-only public header in include/.
project(visual_renderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# No clang-14/libc++-14 package exists on this project's Ubuntu 22.04 dev/
# robot images and there's no root to `apt install` one -- see plan Task 1
# Step 5's fix-up note (docs/superpowers/plans/2026-08-18-visual-mode.md) for
# the root-less acquisition recipe (apt-get download + dpkg-deb -x into a
# local prefix). That prefix's bin/ has to be resolvable as CMAKE_CXX_COMPILER
# (on PATH, or passed as an absolute path) -- CMake can't discover it on its
# own. On this dev box it currently lives at
# ~/.local/opt/clang14-toolchain/bin; the plan doc's Task 1 Step 5 records the
# exact PATH-independent configure command (absolute -DCMAKE_CXX_COMPILER=)
# verified clean-shell-reproducible.
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(FATAL_ERROR
        "visual_renderer must be built with clang++ (got CMAKE_CXX_COMPILER_ID="
        "'${CMAKE_CXX_COMPILER_ID}'). Configure with e.g.:\n"
        "  cmake -DCMAKE_CXX_COMPILER=/path/to/clang14/bin/clang++ "
        "-DCMAKE_CXX_FLAGS=-stdlib=libc++ -DCMAKE_EXE_LINKER_FLAGS=-stdlib=libc++ .. \n"
        "See the plan doc (Task 1 Step 5 fix-up) for how to obtain such a "
        "toolchain root-lessly and this dev box's current path.")
endif()
if(NOT CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    message(FATAL_ERROR
        "visual_renderer must be built against libc++ (pass "
        "-DCMAKE_CXX_FLAGS=-stdlib=libc++ -DCMAKE_EXE_LINKER_FLAGS=-stdlib=libc++). "
        "See spec §2 for why gcc/libstdc++ and clang/libc++ must not mix.")
endif()

# This clang++'s libc++.so.1/libunwind.so.1 live next to it, not on the
# system loader path (ldconfig only knows the system's libunwind.so.8) --
# without this, every binary built here needs LD_LIBRARY_PATH exported by
# hand to run (this bit us: `filament_link_probe` failed with "libc++.so.1:
# cannot open shared object file" under plain ctest). Resolve the runtime dir
# from the compiler itself (never hardcode it -- it's a root-less local
# prefix, not a fixed install path) and bake it in as an rpath so build-tree
# binaries (ctest included) are self-sufficient.
execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=libc++.so
    OUTPUT_VARIABLE _libcxx_so
    OUTPUT_STRIP_TRAILING_WHITESPACE)
get_filename_component(_libcxx_so "${_libcxx_so}" REALPATH)
get_filename_component(_libcxx_runtime_dir "${_libcxx_so}" DIRECTORY)
if(NOT EXISTS "${_libcxx_runtime_dir}/libc++.so.1")
    message(FATAL_ERROR
        "visual_renderer: could not resolve the libc++ runtime dir from "
        "compiler '${CMAKE_CXX_COMPILER}' (looked in '${_libcxx_runtime_dir}' "
        "for libc++.so.1). Is CMAKE_CXX_COMPILER a real clang++ with a "
        "co-located libc++?")
endif()
set(CMAKE_BUILD_RPATH "${_libcxx_runtime_dir}")

# GNU ld defaults to DT_RUNPATH (--enable-new-dtags), which only applies to
# an executable's own *direct* NEEDED entries -- it is NOT consulted while
# resolving libc++.so.1's own NEEDED entry on libunwind.so.1, so the rpath
# above alone still 404s on libunwind.so.1 at runtime (confirmed). The old
# DT_RPATH form is consulted for the whole transitive resolution, which is
# what we actually need here since libunwind.so.1 is never a *direct*
# dependency of anything we build.
add_link_options(-Wl,--disable-new-dtags)

include(cmake/GetFilament.cmake)

# GLOB with CONFIGURE_DEPENDS so later tasks (Epic 0 Task 2+) that add files
# under src/, tests/, examples/ don't need to touch this CMakeLists.txt too —
# a fresh `cmake` re-run picks them up.
file(GLOB_RECURSE VISUAL_RENDERER_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(visual_renderer STATIC ${VISUAL_RENDERER_SOURCES})
add_library(micropilot_visualization::visual_renderer ALIAS visual_renderer)

target_include_directories(visual_renderer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(visual_renderer PUBLIC Filament::filament)

enable_testing()

add_test(NAME check_pod_header
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_pod_header.sh)

# Build-integration proof (Epic 0 Task 1): actually links `visual_renderer`
# (and therefore the ~30 cross-referencing Filament static archives pulled in
# by cmake/GetFilament.cmake) into a real executable and exercises a call
# that spans that archive graph. check_pod_header.sh proves the header is
# POD-clean; this proves the link recipe itself is sound — neither the
# static-lib-only build above nor the header check ever invokes the linker
# against Filament's archives. See smoke/filament_link_probe.cpp.
add_executable(filament_link_probe smoke/filament_link_probe.cpp)
target_link_libraries(filament_link_probe PRIVATE visual_renderer)
add_test(NAME filament_link_probe COMMAND filament_link_probe)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    find_package(GTest QUIET)
    if(GTest_FOUND)
        file(GLOB _visual_renderer_test_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")
        foreach(_test_src ${_visual_renderer_test_srcs})
            get_filename_component(_test_name "${_test_src}" NAME_WE)
            add_executable(${_test_name} "${_test_src}")
            target_link_libraries(${_test_name} PRIVATE visual_renderer GTest::GTest GTest::Main)
            include(GoogleTest)
            gtest_discover_tests(${_test_name} DISCOVERY_TIMEOUT 30)
        endforeach()
    else()
        message(STATUS "visual_renderer: GTest not found; skipping lib tests.")
    endif()
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    file(GLOB _visual_renderer_example_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp")
    foreach(_example_src ${_visual_renderer_example_srcs})
        get_filename_component(_example_name "${_example_src}" NAME_WE)
        add_executable(${_example_name} "${_example_src}")
        target_link_libraries(${_example_name} PRIVATE visual_renderer)
    endforeach()
endif()
