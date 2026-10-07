#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# Fails if configuring with OVERLUME_LLVM_ROOT rewrites the dev clang wrapper.
# Uses a scratch XDG_CACHE_HOME and a fake LLVM root; needs no real compiler.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
export XDG_CACHE_HOME="$tmp/cache"
dev="$XDG_CACHE_HOME/overlume-toolchain-cesium/root/usr/lib/llvm-18/bin"
mkdir -p "$dev" "$tmp/llvm/bin"
touch "$dev/clang++" "$dev/clang" "$tmp/llvm/bin/clang++" "$tmp/llvm/bin/clang"
inc="include($here/overlume/cmake/clang18-toolchain-common.cmake)"
printf '%s\n' "$inc" >"$tmp/t.cmake"
cmake -P "$tmp/t.cmake"
wrap="$XDG_CACHE_HOME/overlume-toolchain-cesium/wrap/clang++"
before="$(sha256sum "$wrap")"
OVERLUME_LLVM_ROOT="$tmp/llvm" cmake -P "$tmp/t.cmake"
[ "$before" = "$(sha256sum "$wrap")" ] || { echo "FAIL: dev wrapper rewritten"; exit 1; }
grep -q "$tmp/llvm/bin/clang++" "$XDG_CACHE_HOME/overlume-toolchain-cesium/wrap-llvm-release/clang++" \
    || { echo "FAIL: release wrapper missing"; exit 1; }
echo "PASS: wrapper isolation"
