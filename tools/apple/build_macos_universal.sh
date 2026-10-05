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
    local out="${1:?OUT_STAGE}" a="$repo/overlume/stage-macos-arm64" x="$repo/overlume/stage-macos-x86_64" f d base
    [ -d "$a" ] && [ -d "$x" ] || die "need $a and $x"
    # Everything but the numbered static-link archives must list identically.
    diff <(cd "$a" && find . \( -type f -o -type l \) | grep -v '/overlume/deps/' | LC_ALL=C sort) \
         <(cd "$x" && find . \( -type f -o -type l \) | grep -v '/overlume/deps/' | LC_ALL=C sort) \
        || die "the arm64 and x86_64 installs list different files"
    rm -rf "$out"
    cp -R "$x" "$out"   # symlinks stay symlinks; x86_64 is authoritative for the deps numbering
    while IFS= read -r f; do
        case "$f" in ./lib/overlume/deps/*|./lib/cmake/overlume/overlumeStaticTargets.cmake) continue ;; esac
        if is_macho "$a/$f"; then
            lipo -create "$a/$f" "$x/$f" -output "$out/$f"
            lipo "$out/$f" -verify_arch arm64 x86_64 || die "$f is not universal"
        else
            cmp -s "$a/$f" "$x/$f" || die "$f differs between arm64 and x86_64 installs"
        fi
    done < <(cd "$a" && find . -type f)
    # CI 37340637284: the arm64 build links Filament's prebuilt libs (extra bluegl/bluevk, different
    # numbering), the x86_64 build links the source-built ones. Merge the archives by name, numbered
    # as in x86_64 (overlumeStaticTargets.cmake, copied from x86_64, lists them by that numbering);
    # an x86_64-only one is an error.
    for d in "$x"/lib/overlume/deps/*.a; do
        base="$(basename "$d")"; base="${base#???_}"
        set -- "$a"/lib/overlume/deps/???_"$base"
        [ -f "$1" ] || die "x86_64 archive $base has no arm64 counterpart"
        lipo -create "$1" "$d" -output "$out/lib/overlume/deps/$(basename "$d")"
        lipo "$out/lib/overlume/deps/$(basename "$d")" -verify_arch arm64 x86_64 || die "$base is not universal"
    done
    # An arm64-only archive (bluegl/bluevk) is still needed by the arm64 link (CI 37342151897:
    # undefined _bluegl_*), so ship it as a fat archive with an empty x86_64 slice, numbered 9NN so
    # it lands after x86_64's; ld64 does not care about archive order.
    local n=0 extra="" empty="$out/.empty"
    : >"$empty.c"; clang -arch x86_64 -c "$empty.c" -o "$empty.o"; libtool -static -arch_only x86_64 -o "$empty.a" "$empty.o" 2>/dev/null
    for d in "$a"/lib/overlume/deps/*.a; do
        base="$(basename "$d")"; base="${base#???_}"
        ls "$x"/lib/overlume/deps/???_"$base" >/dev/null 2>&1 && continue
        n=$((n + 1)); name="$(printf '9%02d' "$n")_$base"
        lipo -create "$d" "$empty.a" -output "$out/lib/overlume/deps/$name"
        lipo "$out/lib/overlume/deps/$name" -verify_arch arm64 x86_64 || die "$name is not universal"
        extra="$extra;\${_overlume_prefix}/lib/overlume/deps/$name"
    done
    rm -f "$empty".*
    if [ -n "$extra" ]; then
        EXTRA="$extra" perl -0pi -e 's/(.*deps\/\d{3}_[^;"]*\.a)/$1$ENV{EXTRA}/s' \
            "$out/lib/cmake/overlume/overlumeStaticTargets.cmake"
        grep -q '/deps/901_' "$out/lib/cmake/overlume/overlumeStaticTargets.cmake" || die "static targets not patched"
    fi
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
