#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# smoke_macos.sh PREFIX [ARCH] -- clean-room consumer of an installed macOS Overlume (a stage dir or
# the unpacked tar.gz): builds tools/package_smoke against PREFIX (find_package + pkg-config, the
# consumer's own yaml-cpp, theme_assets_dir = nullptr) and with the static component, runs both as
# ARCH (arm64 | x86_64 under Rosetta; default: native). Renders a frame when a Metal device
# exists; otherwise runs --expect-no-gpu and emits ::warning::no Metal device -- which one happened
# is printed. Needs cmake, pkg-config and yaml-cpp (brew install yaml-cpp pkg-config).
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
prefix="$(cd "${1:?PREFIX}" && pwd)"; arch="${2:-$(uname -m)}"
w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
export PKG_CONFIG_PATH="$prefix/share/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
# Homebrew's yaml-cpp is arm64-only: the symbol-clash consumer runs natively only.
yaml=ON; [ "$arch" = "$(uname -m)" ] || yaml=OFF
run() { arch -"$arch" "$@"; }

cmake -S "$repo/tools/package_smoke" -B "$w/shared" "-DCMAKE_PREFIX_PATH=$prefix" "-DCMAKE_OSX_ARCHITECTURES=$arch" "-DSMOKE_CONSUMER_YAML=$yaml" >/dev/null
cmake --build "$w/shared" >/dev/null
cmake -S "$repo/tools/package_smoke" -B "$w/static" "-DCMAKE_PREFIX_PATH=$prefix" "-DCMAKE_OSX_ARCHITECTURES=$arch" -DSMOKE_STATIC=ON >/dev/null
cmake --build "$w/static" >/dev/null

gpu=render
if ! run "$w/shared/package_smoke" --expect-render >"$w/out.txt" 2>&1; then
    if grep -q 'create_renderer returned nullptr' "$w/out.txt"; then
        gpu=nogpu
        echo "::warning::no Metal device on this runner ($arch): smoke ran --expect-no-gpu, no frame was rendered"
    else
        cat "$w/out.txt"; echo "FAIL: shared smoke ($arch)"; exit 1
    fi
fi
for exe in "$w/shared/package_smoke" "$w/shared/package_smoke_pc" "$w/static/package_smoke_static"; do
    if [ $gpu = render ]; then flag=--expect-render; else flag=--expect-no-gpu; fi
    run "$exe" $flag >"$w/out.txt" 2>&1 || { cat "$w/out.txt"; echo "FAIL: $(basename "$exe") ($arch)"; exit 1; }
done
echo "PASS: macOS smoke ($arch, $gpu): shared find_package + pkg-config + static"
