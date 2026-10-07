#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_render_vcpkg_conan.sh -- render_vcpkg_conan.sh on tiny fake archives: the good case renders
# with the right hashes, a tampered archive / missing SUMS entry / missing archive must FAIL.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
r="$repo/tools/release/render_vcpkg_conan.sh"
w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
v=1.2.3
names=(linux-x86_64.tar.gz linux-aarch64.tar.gz macos-universal.tar.gz windows-x64.zip windows-arm64.zip)
mkdir "$w/rel"
for n in "${names[@]}"; do echo "fake $n" > "$w/rel/overlume-$v-$n"; done
(cd "$w/rel" && sha256sum overlume-* > SHA256SUMS-all.txt)
bad() { echo "FAIL: $*"; exit 1; }

"$r" "$w/rel" "$v" "$w/out" http://h:1 > /dev/null || bad "good case did not render"
want="$(sha512sum "$w/rel/overlume-$v-windows-x64.zip" | cut -d' ' -f1)"
grep -q "$want" "$w/out/vcpkg-port/ports/overlume/portfile.cmake" || bad "vcpkg SHA512 not filled"
grep -q "$(sha256sum "$w/rel/overlume-$v-macos-universal.tar.gz" | cut -d' ' -f1)" "$w/out/conan/conandata.yml" || bad "conan SHA256 not filled"
grep -q 'http://h:1/overlume-1.2.3-linux-aarch64.tar.gz' "$w/out/conan/conandata.yml" || bad "BASE_URL not applied"
unzip -l "$w/out/overlume-$v-vcpkg-port.zip" | grep -q 'ports/overlume/portfile.cmake' || bad "port zip layout"
unzip -l "$w/out/overlume-$v-conan-recipe.zip" | grep -q 'test_package/main.cpp' || bad "recipe zip layout"

echo tampered >> "$w/rel/overlume-$v-linux-x86_64.tar.gz"
! "$r" "$w/rel" "$v" "$w/o2" http://h:1 > /dev/null 2>&1 || bad "tampered archive was accepted"
echo "fake linux-x86_64.tar.gz" > "$w/rel/overlume-$v-linux-x86_64.tar.gz"
grep -v 'windows-arm64' "$w/rel/SHA256SUMS-all.txt" > "$w/rel/SHA256SUMS-all.tmp" && mv "$w/rel/SHA256SUMS-all.tmp" "$w/rel/SHA256SUMS-all.txt"
! "$r" "$w/rel" "$v" "$w/o3" http://h:1 > /dev/null 2>&1 || bad "missing SUMS entry was accepted"
(cd "$w/rel" && sha256sum overlume-* > SHA256SUMS-all.txt); rm "$w/rel/overlume-$v-macos-universal.tar.gz"
! "$r" "$w/rel" "$v" "$w/o4" http://h:1 > /dev/null 2>&1 || bad "missing archive was accepted"
echo "PASS: render_vcpkg_conan.sh good + 3 negative cases"
