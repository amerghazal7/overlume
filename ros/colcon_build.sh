#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
set -u

export COLCON_DEFAULTS_FILE="${SCRIPT_DIR}/config_colcon.yaml"

CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
echo "Building ROS packages in ${CMAKE_BUILD_TYPE} mode."

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
