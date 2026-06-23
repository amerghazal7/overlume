#!/bin/bash
# colcon_build.sh [pkg1 pkg2 ...]
# Build the TPSProjector ROS2 packages via colcon.
#
# Mirrors micropilot_manager/scripts/ros_apps_build/colcon_build.sh conventions.
#
# The micropilot_rendering CUDA library is NOT a ROS package.
# We put it on CMAKE_PREFIX_PATH so find_package(micropilot_rendering) resolves
# inside any ament_cmake package that needs it.
#
# Usage:
#   cd cuda/scripts/ros_apps_build
#   ./colcon_build.sh                               # build all packages
#   ./colcon_build.sh micropilot_rendering_node     # build specific package(s)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # cuda/

# ── ROS2 environment ─────────────────────────────────────────────────────────
# Note: ROS setup.bash uses unbound variables internally; disable -u around it.
set +u
source /opt/ros/humble/setup.bash
set -u

# ── Non-ROS rendering lib on CMAKE_PREFIX_PATH ───────────────────────────────
RENDERING_INSTALL="${REPO_ROOT}/install/libs"
export CMAKE_PREFIX_PATH="${RENDERING_INSTALL}:${CMAKE_PREFIX_PATH:-}"

# ── colcon config ─────────────────────────────────────────────────────────────
export COLCON_DEFAULTS_FILE="${SCRIPT_DIR}/config_colcon.yaml"
export COLCON_LOG_PATH="${REPO_ROOT}/logs/build_logs/ros_apps_build"

# ── build type ────────────────────────────────────────────────────────────────
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
echo "Building ROS packages in ${CMAKE_BUILD_TYPE} mode."
echo "CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"

# ── cd to the ros_apps workspace root (where src/ lives) ─────────────────────
cd "${REPO_ROOT}/src/ros_apps"

if [ "$#" -gt 0 ]; then
    echo "Building specified packages: $*"
    colcon build \
        --packages-select "$@" \
        --symlink-install \
        --parallel-workers $(($(nproc)/2)) \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
else
    echo "Building all packages."
    colcon build \
        --symlink-install \
        --parallel-workers $(($(nproc)/2)) \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
fi
