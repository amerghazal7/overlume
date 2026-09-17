#!/bin/bash
# colcon_build.sh [pkg1 pkg2 ...]
# Build the TPSProjector ROS2 packages via colcon.
#
# Usage:
#   cd ros
#   ./colcon_build.sh                               # build all packages
#   ./colcon_build.sh overlume_ros # build specific package(s)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # ros/ -- the colcon workspace root

# ── ROS2 environment ─────────────────────────────────────────────────────────
# Note: ROS setup.bash uses unbound variables internally; disable -u around it.
set +u
source /opt/ros/humble/setup.bash
set -u

# ── colcon config ─────────────────────────────────────────────────────────────
export COLCON_DEFAULTS_FILE="${SCRIPT_DIR}/config_colcon.yaml"
# Build/install/log all use colcon's default bases (build/, install/, log/
# under the cwd below) now that this workspace root has no extra nesting to
# route around.

# ── build type ────────────────────────────────────────────────────────────────
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
echo "Building ROS packages in ${CMAKE_BUILD_TYPE} mode."

# ── cd to the workspace root (where src/ lives) ──────────────────────────────
cd "${SCRIPT_DIR}"

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
