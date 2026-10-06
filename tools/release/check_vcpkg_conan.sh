#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_vcpkg_conan.sh RENDERED_DIR PKG_DIR VARIANT   (VARIANT = shared | static)
# RENDERED_DIR: output of render_vcpkg_conan.sh run with BASE_URL http://127.0.0.1:8000.
# PKG_DIR: the release archives (this platform's tar.gz/zip). Serves PKG_DIR on 127.0.0.1:8000, then
#   - vcpkg install overlume --overlay-ports=... and builds + runs the consumer with the vcpkg toolchain
#   - conan create (runs test_package) with -o overlume/*:shared=True|False
# Needs VCPKG_ROOT (bootstrapped), conan 2 and cmake on PATH. Prints PASS/FAIL lines only.
# On Linux the static variant needs clang >= 18 + libc++ (CC/CXX are set to clang/clang++ here).
set -euo pipefail

if [ $# -ne 3 ]; then echo "usage: $0 RENDERED_DIR PKG_DIR shared|static" >&2; exit 2; fi
rendered="$(cd "$1" && pwd)"; pkgs="$(cd "$2" && pwd)"; variant="$3"
: "${VCPKG_ROOT:?VCPKG_ROOT is not set}"
case "$variant" in shared|static) ;; *) echo "FAIL: variant must be shared|static" >&2; exit 2;; esac
PY=python3; command -v python3 >/dev/null || PY=python
work="$(mktemp -d)"
server=""
cleanup() { [ -n "$server" ] && kill "$server" 2>/dev/null || true; rm -rf "$work"; }
trap cleanup EXIT
fail() { echo "FAIL: $*"; exit 1; }
run() { local step="$1"; shift; "$@" > "$work/step.log" 2>&1 || { tail -n 40 "$work/step.log"; fail "$step"; }; }

host="$(uname -s)-$(uname -m)"
case "$host" in  # native Windows tools get drive-letter paths with forward slashes, never MSYS-converted ones
  MINGW*|MSYS*) work="$(cygpath -m "$work")"; rendered="$(cygpath -m "$rendered")"; pkgs="$(cygpath -m "$pkgs")";;
esac

# The release archives (and their sums) over a loopback server.
(cd "$pkgs" && exec "$PY" -m http.server 8000 --bind 127.0.0.1 > "$work/http.log" 2>&1) &
server=$!
for _ in $(seq 1 50); do curl -fsS -o /dev/null http://127.0.0.1:8000/ 2>/dev/null && break; sleep 0.2; done
curl -fsS -o /dev/null http://127.0.0.1:8000/ || fail "http.server did not start"

conan_args=(-o "overlume/*:shared=$([ "$variant" = shared ] && echo True || echo False)")
cmake_args=()
vcpkg_pkg=overlume
case "$host" in
  Linux-x86_64)  triplet=x64-linux;;
  Linux-aarch64) triplet=arm64-linux;;
  Darwin-arm64)  triplet=arm64-osx;;
  Darwin-x86_64) triplet=x64-osx;;
  MINGW*|MSYS*)  triplet=x64-windows;;
  *) fail "unsupported host $host";;
esac
if [ "$variant" = static ]; then
  cmake_args+=(-DOVERLUME_STATIC=ON)
  case "$host" in
    MINGW*|MSYS*) triplet=x64-windows-static-md;;
    Linux-*)
      vcpkg_pkg='overlume[static]'
      export CC=clang CXX=clang++
      cmake_args+=(-DCMAKE_CXX_FLAGS=-stdlib=libc++)
      conan_args+=(-s compiler=clang -s "compiler.version=$(clang --version | sed -n 's/.*version \([0-9]*\)\..*/\1/p')"
                   -s compiler.libcxx=libc++ -s compiler.cppstd=17);;
    *) vcpkg_pkg='overlume[static]';;
  esac
fi

# ---- vcpkg ------------------------------------------------------------------------------------
export VCPKG_DISABLE_METRICS=1
vcpkg="$VCPKG_ROOT/vcpkg"
run "vcpkg install $vcpkg_pkg:$triplet" "$vcpkg" install "$vcpkg_pkg" --triplet "$triplet" \
    --overlay-ports="$rendered/vcpkg-port/ports" --x-install-root="$work/vcpkg_installed" \
    --x-buildtrees-root="$work/buildtrees" --x-packages-root="$work/packages" \
    --downloads-root="$work/downloads" --binarysource=clear
inst="$work/vcpkg_installed/$triplet"
[ -f "$inst/share/overlume/copyright" ] && [ -d "$inst/share/overlume/themes" ] || fail "vcpkg install is missing copyright/themes"
run "vcpkg consumer configure" cmake -S "$rendered/conan/test_package" -B "$work/vb" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET="$triplet" \
    -DVCPKG_MANIFEST_MODE=OFF -DVCPKG_INSTALLED_DIR="$work/vcpkg_installed" "${cmake_args[@]}"
run "vcpkg consumer build" cmake --build "$work/vb" --config Release
exe="$(find "$work/vb" -type f \( -name consumer -o -name consumer.exe \) | head -n 1)"
[ -n "$exe" ] || fail "vcpkg consumer binary not built"
"$exe" > "$work/run.log" 2>&1 || { cat "$work/run.log"; fail "vcpkg consumer run"; }
grep -q '^overlume 0\.1\.0 renderer=' "$work/run.log" || { cat "$work/run.log"; fail "vcpkg consumer output"; }
echo "PASS: vcpkg $vcpkg_pkg:$triplet ($(tail -n 1 "$work/run.log"))"

# ---- Conan ------------------------------------------------------------------------------------
export CONAN_HOME="$work/conan_home"
run "conan profile detect" conan profile detect --force
ver="$(sed -n 's/^  "\([0-9.]*\)":$/\1/p' "$rendered/conan/conandata.yml")"
cp -R "$rendered/conan" "$work/conan"  # test_package writes its build folder next to itself
run "conan create ($variant)" conan create "$work/conan" --version "$ver" "${conan_args[@]}" --build=missing
echo "PASS: conan create overlume/$ver ($variant, test_package ran)"
