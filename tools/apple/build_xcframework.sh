#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# iOS XCFrameworks, in steps so CI can build the three slices as parallel jobs:
#   build_xcframework.sh slice device-arm64|sim-arm64|sim-x86_64
#       -> overlume/build-ios-<slice>, installed to overlume/stage-ios-<slice> (shared + static)
#   build_xcframework.sh assemble OUTDIR
#       -> OUTDIR/Overlume.xcframework       (dynamic: Overlume.framework per platform)
#          OUTDIR/OverlumeStatic.xcframework (one merged liboverlume.a per platform)
#   build_xcframework.sh zip OUTDIR
#       -> OUTDIR/Overlume-<ver>.xcframework.zip, OUTDIR/OverlumeStatic-<ver>.xcframework.zip (after signing)
#   build_xcframework.sh smoke XCFRAMEWORK_DIR
#       -> XCTest in the iOS simulator through tools/apple/ios_smoke (a SwiftPM consumer of the
#          xcframework, so it also proves the binaryTarget resolves and links)
# Slices: device = iphoneos arm64; simulator = iphonesimulator arm64 + x86_64, lipo'd into one. Prints
# PASS/FAIL only.
set -euo pipefail
# CMake 4 (Homebrew on the runners) rejects the < 3.5 minimums of pinned third-party projects.
export CMAKE_POLICY_VERSION_MINIMUM=3.5
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
die() { echo "FAIL: $*" >&2; exit 1; }
version() {
    local f="$repo/overlume/include/overlume/version.h" k
    for k in MAJOR MINOR PATCH; do sed -n "s/^#define OVERLUME_VERSION_$k \([0-9]*\)\$/\1/p" "$f"; done | paste -sd. -
}

slice() {
    local name="${1:?slice}" sysroot arch
    case "$name" in
        device-arm64) sysroot=iphoneos; arch=arm64 ;;
        sim-arm64) sysroot=iphonesimulator; arch=arm64 ;;
        sim-x86_64) sysroot=iphonesimulator; arch=x86_64 ;;
        *) die "unknown slice '$name'" ;;
    esac
    local b="$repo/overlume/build-ios-$name" st="$repo/overlume/stage-ios-$name"
    # shellcheck disable=SC2086
    cmake -S "$repo/overlume" -B "$b" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_SYSTEM_NAME=iOS "-DCMAKE_OSX_SYSROOT=$sysroot" "-DCMAKE_OSX_ARCHITECTURES=$arch" \
        -DOVERLUME_ENABLE_CESIUM=ON -DOVERLUME_BUILD_DOCS=OFF -DOVERLUME_BUILD_EXAMPLES=OFF \
        ${OVERLUME_APPLE_CMAKE_ARGS:-}
    # Libraries only: iOS test executables cannot run on the build host.
    cmake --build "$b" --target overlume_shared overlume -- -k 0
    rm -rf "$st"
    cmake --install "$b" --prefix "$st"
    echo "PASS: built iOS slice $name ($st)"
}

dylib_of() { find "$1/lib" -maxdepth 1 -name "liboverlume.*.*.*.dylib" | head -1; }

# make_framework STAGE_PREFIX DYLIB OUTDIR -- Overlume.framework (flat iOS layout) under OUTDIR.
make_framework() {
    local st="$1" dylib="$2" fw="$3/Overlume.framework" v
    v="$(version)"
    rm -rf "$fw"; mkdir -p "$fw/Headers" "$fw/Resources"
    cp "$dylib" "$fw/Overlume"
    install_name_tool -id @rpath/Overlume.framework/Overlume "$fw/Overlume"
    # Flat headers, so <Overlume/api.h> works and scene.h's own include resolves beside it.
    cp "$st"/include/overlume/*.h "$fw/Headers/"
    sed -i '' 's|#include "overlume/api.h"|#include "api.h"|' "$fw/Headers/scene.h"
    cp -R "$st/share/overlume/themes" "$fw/Resources/themes"
    cp -R "$st/share/overlume/models" "$fw/Resources/models"
    cat > "$fw/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>CFBundleExecutable</key><string>Overlume</string>
<key>CFBundleIdentifier</key><string>io.github.amerghazal7.overlume</string>
<key>CFBundleName</key><string>Overlume</string>
<key>CFBundlePackageType</key><string>FMWK</string>
<key>CFBundleShortVersionString</key><string>$v</string>
<key>CFBundleVersion</key><string>$v</string>
<key>MinimumOSVersion</key><string>15.0</string>
</dict></plist>
PLIST
}

# merged_static STAGE_PREFIX OUT.a -- liboverlume.a plus every dependency archive, one library.
merged_static() {
    local st="$1" out="$2" arch="${3:-}" only=()
    # CI 37150239822: the x86_64 simulator merge came out arm64 only; pin the architecture.
    [ -z "$arch" ] || only=(-arch_only "$arch")
    libtool -static "${only[@]}" -o "$out" "$st/lib/liboverlume.a" "$st"/lib/overlume/deps/*.a
}

assemble() {
    local outdir="${1:?OUTDIR}" v w dev ar x
    v="$(version)"; mkdir -p "$outdir"; outdir="$(cd "$outdir" && pwd)"
    dev="$repo/overlume/stage-ios-device-arm64"
    ar="$repo/overlume/stage-ios-sim-arm64"; x="$repo/overlume/stage-ios-sim-x86_64"
    for d in "$dev" "$ar" "$x"; do [ -d "$d" ] || die "missing $d"; done
    w="$(mktemp -d)"; trap 'rm -rf "$w"' RETURN
    mkdir -p "$w/device" "$w/sim"

    # ---- dynamic ----
    make_framework "$dev" "$(dylib_of "$dev")" "$w/device"
    make_framework "$ar" "$(dylib_of "$ar")" "$w/sim"
    lipo -create "$(dylib_of "$ar")" "$(dylib_of "$x")" -output "$w/sim/Overlume.framework/Overlume"
    install_name_tool -id @rpath/Overlume.framework/Overlume "$w/sim/Overlume.framework/Overlume"
    lipo "$w/sim/Overlume.framework/Overlume" -verify_arch arm64 x86_64 || die "simulator framework is not arm64+x86_64"
    # The export hygiene check on each platform's binary.
    cp "$w/device/Overlume.framework/Overlume" "$w/chk-device"
    lipo -thin arm64 "$w/sim/Overlume.framework/Overlume" -output "$w/chk-sim-arm64"
    lipo -thin x86_64 "$w/sim/Overlume.framework/Overlume" -output "$w/chk-sim-x86_64"
    for f in "$w"/chk-*; do
        "$repo/overlume/scripts/check_shared_exports.sh" nm otool "$f" || die "export check failed for $(basename "$f")"
    done
    rm -rf "$outdir/Overlume.xcframework" "$outdir/OverlumeStatic.xcframework"
    xcodebuild -create-xcframework -framework "$w/device/Overlume.framework" \
        -framework "$w/sim/Overlume.framework" -output "$outdir/Overlume.xcframework"

    # ---- static ----
    merged_static "$dev" "$w/device/liboverlume.a" arm64
    merged_static "$ar" "$w/sim/liboverlume-arm64.a" arm64
    merged_static "$x" "$w/sim/liboverlume-x86_64.a" x86_64
    lipo -create "$w/sim/liboverlume-arm64.a" "$w/sim/liboverlume-x86_64.a" -output "$w/sim/liboverlume.a"
    xcodebuild -create-xcframework -library "$w/device/liboverlume.a" -headers "$dev/include" \
        -library "$w/sim/liboverlume.a" -headers "$ar/include" -output "$outdir/OverlumeStatic.xcframework"
    echo "PASS: xcframeworks in $outdir"
}

zip_xcframeworks() {
    local outdir="${1:?OUTDIR}" v
    v="$(version)"
    ( cd "$outdir"
      rm -f "Overlume-$v.xcframework.zip" "OverlumeStatic-$v.xcframework.zip"
      ditto -c -k --sequesterRsrc --keepParent Overlume.xcframework "Overlume-$v.xcframework.zip"
      ditto -c -k --sequesterRsrc --keepParent OverlumeStatic.xcframework "OverlumeStatic-$v.xcframework.zip" )
    echo "PASS: zipped xcframeworks ($v)"
}

smoke() {
    local xcf="${1:?XCFRAMEWORK_DIR (holding Overlume.xcframework)}" pkg="$repo/tools/apple/ios_smoke" udid log
    xcf="$(cd "$xcf" && pwd)"
    rm -rf "$pkg/Overlume.xcframework" "$pkg/.build"
    cp -R "$xcf/Overlume.xcframework" "$pkg/Overlume.xcframework"
    udid="$(xcrun simctl list devices available -j | python3 -c '
import json, sys
d = json.load(sys.stdin)["devices"]
for rt in sorted(d, reverse=True):
    for dev in d[rt]:
        if dev["name"].startswith("iPhone") and "iOS" in rt:
            print(dev["udid"]); sys.exit(0)
sys.exit(1)')" || die "no iPhone simulator available"
    xcrun simctl boot "$udid" 2>/dev/null || true
    log="$(mktemp)"
    # CI 37333934653: neither OverlumeSmoke-Package nor OverlumeShim exists as a scheme; ask xcodebuild.
    scheme="$( cd "$pkg" && xcodebuild -list -json 2>/dev/null | python3 -c '
import json, sys
s = json.load(sys.stdin).get("workspace", {}).get("schemes", [])
print("OverlumeSmoke-Package" if "OverlumeSmoke-Package" in s else (s[0] if s else ""))' )"
    ( cd "$pkg" && xcodebuild -list ) || true
    [ -n "$scheme" ] || die "xcodebuild lists no scheme for the smoke package"
    if [ -n "${GITHUB_ACTIONS:-}" ] && [ "${OVERLUME_SMOKE_RENDER:-0}" != 1 ]; then
        echo "::warning::hosted runner: simulator render skipped (paravirtual GPU); run once on a real Mac before the first tagged release"
        export TEST_RUNNER_OVERLUME_SMOKE_SKIP_RENDER=1
    fi
    ( cd "$pkg" && xcodebuild test -scheme "$scheme" -destination "id=$udid" ) >"$log" 2>&1 \
        || { tail -n 60 "$log"; die "iOS simulator XCTest failed"; }
    grep -E 'OVERLUME_SMOKE|Test Suite .* (passed|failed)|Executed [0-9]+ tests' "$log" | tail -n 8
    echo "PASS: iOS simulator smoke"
}

case "${1:-}" in
    slice) shift; slice "$@" ;;
    assemble) shift; assemble "$@" ;;
    zip) shift; zip_xcframeworks "$@" ;;
    smoke) shift; smoke "$@" ;;
    *) echo "usage: $0 slice NAME | assemble OUTDIR | zip OUTDIR | smoke XCFRAMEWORK_DIR" >&2; exit 2 ;;
esac
