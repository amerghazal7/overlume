#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# render_vcpkg_conan.sh RELEASE_DIR VERSION OUT_DIR [BASE_URL]
# Fills packaging/vcpkg + packaging/conan from the SHA256SUMS-*.txt files in RELEASE_DIR and
# computes SHA512 (vcpkg) of the archives found there. Writes
#   OUT_DIR/overlume-VERSION-vcpkg-port.zip   (ports/overlume/...)
#   OUT_DIR/overlume-VERSION-conan-recipe.zip (conanfile.py, conandata.yml, test_package/)
# and the same trees unpacked under OUT_DIR/vcpkg-port and OUT_DIR/conan.
# BASE_URL defaults to the GitHub release download URL of v VERSION; the channel checks pass
# http://127.0.0.1:PORT to point both channels at the run's own artifacts.
set -euo pipefail

if [ $# -lt 3 ] || [ $# -gt 4 ]; then echo "usage: $0 RELEASE_DIR VERSION OUT_DIR [BASE_URL]" >&2; exit 2; fi
rel="$(cd "$1" && pwd)"; ver="$2"; out="$3"
base="${4:-https://github.com/amerghazal7/overlume/releases/download/v${ver}}"
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ "$ver" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo "FAIL: bad version '$ver'" >&2; exit 2; }

# key -> archive name (must match the Task 3/7/8 package names exactly)
declare -A archive=(
  [LINUX_X64]="overlume-${ver}-linux-x86_64.tar.gz"
  [LINUX_ARM64]="overlume-${ver}-linux-aarch64.tar.gz"
  [MACOS]="overlume-${ver}-macos-universal.tar.gz"
  [WIN_X64]="overlume-${ver}-windows-x64.zip"
  [WIN_ARM64]="overlume-${ver}-windows-arm64.zip"
)

sub=()
for key in "${!archive[@]}"; do
  name="${archive[$key]}"
  sha256="$(cat "$rel"/SHA256SUMS-*.txt | awk -v n="$name" '$2 == n {print $1}')"
  [[ "$sha256" =~ ^[0-9a-f]{64}$ ]] || { echo "FAIL: no SHA256 for $name in $rel/SHA256SUMS-*.txt" >&2; exit 1; }
  [ -f "$rel/$name" ] || { echo "FAIL: $rel/$name missing (needed for the vcpkg SHA512)" >&2; exit 1; }
  [ "$(sha256sum "$rel/$name" | cut -d' ' -f1)" = "$sha256" ] || { echo "FAIL: $name does not match its SHA256SUMS entry" >&2; exit 1; }
  sub+=(-e "s|@SHA256_${key}@|${sha256}|g" -e "s|@SHA512_${key}@|$(sha512sum "$rel/$name" | cut -d' ' -f1)|g")
done
sub+=(-e "s|@VERSION@|${ver}|g" -e "s|@BASE_URL@|${base}|g")

rm -rf "$out/vcpkg-port" "$out/conan"
mkdir -p "$out/vcpkg-port/ports/overlume" "$out/conan/test_package"
render() { sed "${sub[@]}" "$1" > "$2"; if grep -q '@[A-Z0-9_]*@' "$2"; then echo "FAIL: unfilled placeholder in $2" >&2; exit 1; fi; }
p="$repo/packaging/vcpkg/ports/overlume"
render "$p/vcpkg.json.in" "$out/vcpkg-port/ports/overlume/vcpkg.json"
render "$p/portfile.cmake.in" "$out/vcpkg-port/ports/overlume/portfile.cmake"
cp "$p/usage" "$out/vcpkg-port/ports/overlume/usage"
c="$repo/packaging/conan"
render "$c/conandata.yml.in" "$out/conan/conandata.yml"
cp "$c/conanfile.py" "$out/conan/"
cp "$c"/test_package/* "$out/conan/test_package/"

# python3 -m zipfile is portable (no zip(1) on macOS/Windows runners); run in the tree so names are relative.
(cd "$out/vcpkg-port" && rm -f "../overlume-${ver}-vcpkg-port.zip" && python3 -m zipfile -c "../overlume-${ver}-vcpkg-port.zip" ports)
(cd "$out/conan" && rm -f "../overlume-${ver}-conan-recipe.zip" && python3 -m zipfile -c "../overlume-${ver}-conan-recipe.zip" conanfile.py conandata.yml test_package)
echo "PASS: rendered vcpkg port + conan recipe for ${ver} (base ${base%%/overlume*}/...)"
