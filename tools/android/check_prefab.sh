#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_prefab.sh AAR -- run Google's Prefab CLI (the tool AGP uses) over the AAR for every ABI
# it holds and both consumer STLs: the package must resolve for c++_shared and c++_static apps.
# Needs java (11+); the CLI jar is fetched once into ${OVERLUME_ANDROID_HOME:-~/.cache/overlume-android}/dl
# and pinned by SHA256.
set -euo pipefail
aar="$(cd "$(dirname "${1:?usage: $0 AAR}")" && pwd)/$(basename "$1")"
home="${OVERLUME_ANDROID_HOME:-$HOME/.cache/overlume-android}"
jar="$home/dl/prefab-cli-2.1.0.jar"
sha=e219c8cd6bfd9ff71503a57c6af34b9ba060f03525cc3e58330dee53245a5ed6
mkdir -p "$home/dl"
[ -f "$jar" ] || curl -fsSL -o "$jar" https://dl.google.com/dl/android/maven2/com/google/prefab/cli/2.1.0/cli-2.1.0-all.jar
echo "$sha  $jar" | sha256sum -c - >/dev/null || { echo "FAIL: prefab cli sha256"; exit 1; }
work="$(mktemp -d)"; trap 'rm -rf "$work"' EXIT
unzip -q "$aar" -d "$work/aar"
for d in "$work"/aar/prefab/modules/overlume/libs/android.*; do
    abi="${d##*/android.}"
    for stl in c++_shared c++_static; do
        out="$work/out-$abi-$stl"
        if java -jar "$jar" --build-system cmake --platform android --abi "$abi" --os-version 26 \
                --stl "$stl" --ndk-version 27 --output "$out" "$work/aar/prefab" >"$work/log" 2>&1; then
            echo "PASS: prefab resolves ($abi, consumer $stl)"
            # The static target must carry the system libs (Prefab v2: an android.export_libraries
            # override replaces the top-level list), else Gradle consumers fail to link.
            grep -rh INTERFACE_LINK_LIBRARIES "$out" | grep -q -- '-lEGL;-lGLESv3;-landroid;-llog;-ldl' \
                || { echo "FAIL: static target exports no system libs ($abi, consumer $stl)"; exit 1; }
        else
            cat "$work/log"; echo "FAIL: prefab ($abi, consumer $stl)"; exit 1
        fi
    done
done
