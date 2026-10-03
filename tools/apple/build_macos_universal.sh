#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# macOS universal2 (arm64 + x86_64) build, in steps so CI can run the two arches as parallel jobs
# (the cesium-native vcpkg tree is per-arch, so each arch has its own build dir):
#   build_macos_universal.sh build ARCH            -> overlume/build-macos-ARCH, install to overlume/stage-macos-ARCH
#   build_macos_universal.sh merge OUT_STAGE       -> lipo overlume/stage-macos-{arm64,x86_64} into OUT_STAGE
#   build_macos_universal.sh package STAGE OUTDIR  -> overlume-<ver>-macos-universal.{tar.gz,pkg}
# Prefix of the payload is /usr/local (pkg) or relocatable (tar.gz). Prints PASS/FAIL only.
set -euo pipefail
# CMake 4 (Homebrew on the runners) rejects the < 3.5 minimums of pinned third-party projects.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
die() { echo "FAIL: $*" >&2; exit 1; }
version() {
    local f="$repo/overlume/include/overlume/version.h" k
    for k in MAJOR MINOR PATCH; do sed -n "s/^#define OVERLUME_VERSION_$k \([0-9]*\)\$/\1/p" "$f"; done | paste -sd. -
}

build() {
    local arch="${1:?ARCH (arm64|x86_64)}" b="$repo/overlume/build-macos-$1" st="$repo/overlume/stage-macos-$1"
    # shellcheck disable=SC2086
    cmake -S "$repo/overlume" -B "$b" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        "-DCMAKE_OSX_ARCHITECTURES=$arch" -DOVERLUME_ENABLE_CESIUM=ON \
        -DOVERLUME_BUILD_DOCS=OFF -DOVERLUME_BUILD_EXAMPLES=OFF ${OVERLUME_APPLE_CMAKE_ARGS:-}
    cmake --build "$b" -- -k 0   # report every compile error, not just the first
    rm -rf "$st"
    cmake --install "$b" --prefix "$st"
    echo "PASS: built $arch ($st)"
}

is_macho() { file -b "$1" | grep -qE 'Mach-O|current ar archive'; }

merge() {
    local out="${1:?OUT_STAGE}" a="$repo/overlume/stage-macos-arm64" x="$repo/overlume/stage-macos-x86_64" f
    [ -d "$a" ] && [ -d "$x" ] || die "need $a and $x"
    diff <(cd "$a" && find . \( -type f -o -type l \) | LC_ALL=C sort) \
         <(cd "$x" && find . \( -type f -o -type l \) | LC_ALL=C sort) >/dev/null \
        || die "the arm64 and x86_64 installs list different files"
    rm -rf "$out"
    cp -R "$a" "$out"   # symlinks stay symlinks
    while IFS= read -r f; do
        if is_macho "$a/$f"; then
            lipo -create "$a/$f" "$x/$f" -output "$out/$f"
            lipo "$out/$f" -verify_arch arm64 x86_64 || die "$f is not universal"
        else
            cmp -s "$a/$f" "$x/$f" || die "$f differs between arm64 and x86_64 installs"
        fi
    done < <(cd "$a" && find . -type f)
    echo "PASS: universal2 stage $out"
}

package() {
    local stage="${1:?STAGE}" outdir="${2:?OUTDIR}" v id=io.github.amerghazal7.overlume tmp
    v="$(version)"; mkdir -p "$outdir"; outdir="$(cd "$outdir" && pwd)"
    tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' RETURN
    # tar.gz: prefix-relative, both components (the Homebrew formula installs it as is).
    tar -czf "$outdir/overlume-$v-macos-universal.tar.gz" -C "$stage" .
    pkgbuild --root "$stage" --identifier "$id" --version "$v" --install-location /usr/local \
        "$tmp/overlume.pkg" >/dev/null
    productbuild --package "$tmp/overlume.pkg" "$outdir/overlume-$v-macos-universal.pkg" >/dev/null
    pkgutil --payload-files "$outdir/overlume-$v-macos-universal.pkg" | grep -q 'lib/liboverlume' \
        || die "pkg payload lacks liboverlume"
    echo "PASS: packaged overlume-$v-macos-universal.{tar.gz,pkg} in $outdir"
}

case "${1:-}" in
    build) shift; build "$@" ;;
    merge) shift; merge "$@" ;;
    package) shift; package "$@" ;;
    version) version ;;
    *) echo "usage: $0 build ARCH | merge OUT_STAGE | package STAGE OUTDIR" >&2; exit 2 ;;
esac
