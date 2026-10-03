#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_consumer.sh UNZIPPED_DIR [ABI...] -- build tools/package_smoke/android with the NDK against
# each ABI of the unpacked overlume-<ver>-android.zip (shared + static components must link).
# With RUN_ON_DEVICE=1 (an emulator/device matching the ABI is attached) both consumers also run
# there with --expect-render; the shared one passes no theme dir, so it proves the installed-package
# lookup (<module>/../share/overlume/themes) on Android.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
: "${ANDROID_NDK_HOME:?ANDROID_NDK_HOME is not set}"
root="$(cd "${1:?usage: $0 UNZIPPED_DIR [ABI...]}" && pwd)"; shift
abis=("$@"); [ ${#abis[@]} -gt 0 ] || abis=($(ls "$root"))
for abi in "${abis[@]}"; do
    b="$(mktemp -d)"
    dev=/data/local/tmp/ovc
    cmake -S "$repo/tools/package_smoke/android" -B "$b" -G Ninja \
        -DOVERLUME_SMOKE_THEMES_DIR="$dev/prefix/share/overlume/themes" \
        --toolchain "$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake" \
        -DANDROID_ABI="$abi" -DANDROID_PLATFORM=26 -DANDROID_STL=c++_static \
        -DCMAKE_BUILD_TYPE=Release -DOVERLUME_PREFIX="$root/$abi" >/dev/null
    if cmake --build "$b" >"$b/log" 2>&1; then echo "PASS: consumer links ($abi, shared + static)"
    else tail -30 "$b/log"; echo "FAIL: consumer link ($abi)"; exit 1; fi
    if [ -n "${RUN_ON_DEVICE:-}" ]; then
        adb="${ADB:-adb}"
        "$adb" shell "rm -rf $dev; mkdir -p $dev/bin $dev/prefix/lib"
        "$adb" push "$root/$abi/share" "$dev/prefix/" >/dev/null
        "$adb" push "$root/$abi/lib/liboverlume.so" "$dev/prefix/lib/" >/dev/null
        "$adb" push "$b/smoke_shared" "$b/smoke_static" "$dev/bin/" >/dev/null
        for exe in smoke_shared smoke_static; do
            out="$("$adb" shell "cd $dev && LD_LIBRARY_PATH=$dev/prefix/lib ./bin/$exe --expect-render 2>&1; echo rc=\$?" | tr -d '\r')"
            if [ "$(printf '%s\n' "$out" | tail -1)" = rc=0 ] && printf '%s\n' "$out" | grep -q '^PASS'; then
                echo "PASS: $exe renders on the device ($abi)"
            else printf '%s\n' "$out" | tail -5; echo "FAIL: $exe on the device ($abi)"; exit 1; fi
        done
    fi
    rm -rf "$b"
done
