cmake_minimum_required(VERSION 3.20)

# Standalone example: demonstrates headless find_package usage of
# micropilot_rendering::reprojector. No GL, no ROS, no display.
#
# Build against the install tree:
#   cmake -S . -B build_demo \
#     -Dmicropilot_rendering_DIR=<install_prefix>/lib/cmake/micropilot_rendering
#   cmake --build build_demo
#
# The micropilot_rendering_DIR can also be set via the environment or
# by adding the install prefix to CMAKE_PREFIX_PATH.

project(render_demo_example VERSION 1.0.0 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)

# Locate the installed library.
find_package(micropilot_rendering REQUIRED)

add_executable(render_demo render_demo.cpp)

# Link ONLY against the reprojector (which transitively pulls in CUDA::cudart).
# No GL, no window, no ROS.
target_link_libraries(render_demo PRIVATE micropilot_rendering::reprojector)

# The installed headers live under rendering_reprojector/include/rendering_reprojector/
# in the install prefix.  The target's INTERFACE_INCLUDE_DIRECTORIES already
# covers that, so no extra include_directories() call is needed.

install(TARGETS render_demo RUNTIME DESTINATION .)
