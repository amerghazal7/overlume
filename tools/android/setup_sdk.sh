#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# setup_sdk.sh [ndk|all] -- rootless Android toolchain under ${OVERLUME_ANDROID_HOME:-~/.cache/overlume-android}:
#   ndk  NDK r27c only (what the CI build containers mount)
#   all  + cmdline-tools, platform-tools, emulator and the API 30 google_apis x86_64/x86 system images
# NDK / cmdline-tools archives are pinned by SHA256 (Google publishes SHA-1 only; the SHA-1 below is
# the published one, the SHA256 was recorded from the verified download). sdkmanager checks its own
# packages. Prints PASS/FAIL only.
set -euo pipefail
what="${1:-all}"
home="${OVERLUME_ANDROID_HOME:-$HOME/.cache/overlume-android}"
base=https://dl.google.com/android/repository
ndk_zip=android-ndk-r27c-linux.zip
ndk_sha1=090e8083a715fdb1a3e402d0763c388abb03fb4e
ndk_sha256=59c2f6dc96743b5daf5d1626684640b20a6bd2b1d85b13156b90333741bad5cc
cmd_zip=commandlinetools-linux-9862592_latest.zip   # v10.0, runs on JDK 11
cmd_sha1=a787f0c35d3970f2ad634f7d91a2e7bc49e2c7f5
cmd_sha256=177e20e12637e8e3ba2cbe5fe20b780c38c92bdd85a1633300bdc041e20b2036
mkdir -p "$home/dl"

fetch() {  # file sha1 sha256
    local f="$home/dl/$1"
    [ -f "$f" ] || { curl -fsSL -o "$f.part" "$base/$1" && mv "$f.part" "$f"; }
    echo "$2  $f" | sha1sum -c - >/dev/null || { echo "FAIL: $1 sha1"; exit 1; }
    echo "$3  $f" | sha256sum -c - >/dev/null || { echo "FAIL: $1 sha256"; exit 1; }
}

if [ ! -x "$home/android-ndk-r27c/toolchains/llvm/prebuilt/linux-x86_64/bin/clang" ]; then
    fetch "$ndk_zip" "$ndk_sha1" "$ndk_sha256"
    unzip -q "$home/dl/$ndk_zip" -d "$home"
fi
echo "PASS: NDK r27c at $home/android-ndk-r27c"
[ "$what" = ndk ] && exit 0

sdk="$home/sdk"
if [ ! -x "$sdk/cmdline-tools/10.0/bin/sdkmanager" ]; then
    fetch "$cmd_zip" "$cmd_sha1" "$cmd_sha256"
    mkdir -p "$sdk/cmdline-tools"
    unzip -q "$home/dl/$cmd_zip" -d "$sdk/cmdline-tools"
    mv "$sdk/cmdline-tools/cmdline-tools" "$sdk/cmdline-tools/10.0"
fi
sm="$sdk/cmdline-tools/10.0/bin/sdkmanager"
yes | "$sm" --sdk_root="$sdk" --licenses >/dev/null 2>&1 || true
"$sm" --sdk_root="$sdk" platform-tools emulator \
    "system-images;android-30;google_apis;x86_64" "system-images;android-30;google_apis;x86" \
    >/dev/null
echo "PASS: SDK at $sdk (adb $("$sdk/platform-tools/adb" --version | head -1))"
