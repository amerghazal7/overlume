#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_all_abis.sh [ABI...] -- configure + build overlume (shared + static, Cesium ON) with the
# NDK for each ABI (default: arm64-v8a armeabi-v7a x86_64 x86) into overlume/build-android-<abi>.
# Needs ANDROID_NDK_HOME (r27c; tools/android/setup_sdk.sh provisions it) and a host LLVM for
# Filament's host tools + vcpkg's host triplet (OVERLUME_LLVM_ROOT, or the dev toolchain).
# Extra cmake arguments: OVERLUME_ANDROID_CMAKE_ARGS (e.g. -DOVERLUME_BUILD_EXAMPLES=OFF).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
: "${ANDROID_NDK_HOME:?ANDROID_NDK_HOME is not set (tools/android/setup_sdk.sh)}"
export ANDROID_NDK_HOME
abis=("$@"); [ ${#abis[@]} -gt 0 ] || abis=(arm64-v8a armeabi-v7a x86_64 x86)
for abi in "${abis[@]}"; do
    b="$repo/overlume/build-android-$abi"
    echo "=== $abi -> $b"
    # shellcheck disable=SC2086
    cmake -S "$repo/overlume" -B "$b" -G Ninja \
        --toolchain "$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake" \
        -DANDROID_ABI="$abi" -DANDROID_PLATFORM=26 -DANDROID_STL=c++_static \
        -DCMAKE_BUILD_TYPE=Release -DOVERLUME_ENABLE_CESIUM=ON \
        "-DCMAKE_C_FLAGS_RELEASE=-O2 -DNDEBUG -g0" "-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG -g0" \
        -DOVERLUME_BUILD_DOCS=OFF -DOVERLUME_BUILD_EXAMPLES=OFF \
        -DOVERLUME_TEST_DATA_DIR_RUNTIME=/data/local/tmp/ov/data \
        -DOVERLUME_TEST_TMP_DIR_RUNTIME=/data/local/tmp/ov/tmp \
        -DCMAKE_CROSSCOMPILING_EMULATOR="$here/adb_run.sh" \
        ${OVERLUME_ANDROID_CMAKE_ARGS:-}
    cmake --build "$b"
done
