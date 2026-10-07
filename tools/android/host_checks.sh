#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# host_checks.sh BUILD_DIR -- the tests of an Android build that need no device: POD header,
# static-link flags, Cesium triplet mapping and the shared-library export/dependency check (with the
# NDK's llvm-nm/llvm-readelf). Run directly rather than through ctest: ctest lists the gtest cases of
# a cross build by running the binaries on a device when it loads, which does not exist here.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$(cd "$here/../../overlume" && pwd)"
b="$(cd "${1:?usage: $0 BUILD_DIR}" && pwd)"
cache() { sed -n "s/^$1:[A-Z]*=//p" "$b/CMakeCache.txt"; }
"$src/scripts/check_pod_header.sh"
"$(cache CMAKE_COMMAND)" -P "$src/tests/cmake/test_static_link_flags.cmake"
"$(cache CMAKE_COMMAND)" -P "$src/scripts/check_cesium_triplets.cmake"
"$src/scripts/check_shared_exports.sh" "$(cache CMAKE_NM)" "$(cache CMAKE_READELF)" "$b/liboverlume.so"
echo "PASS: android host checks ($b)"
