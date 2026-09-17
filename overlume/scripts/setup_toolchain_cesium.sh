#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# setup_toolchain_cesium.sh -- root-less bootstrap of Overlume's PRIMARY
# toolchain: clang-18/libc++-18 from apt.llvm.org's jammy channel, unpacked
# into ~/.cache/overlume-toolchain-cesium without sudo. Every part of the
# library (its own C++17 code, yaml-cpp, GoogleTest, the examples) and the
# cesium-native + vcpkg dependency build use this one toolchain;
# cmake/toolchain-clang-libcxx.cmake resolves only this prefix.
#
# History: the "_cesium" in the name is from VM-061, when this was a SECOND
# toolchain used only for cesium-native, whose vcpkg dependency "ada-url"
# requires C++20 `std::ranges::replace` — a ranges <algorithm> overload
# libc++-14/15 do not implement (verified) while libc++-18 does. VM-061
# Step 6 (user decision 2026-09-15) then migrated the whole library from
# clang-14 to this clang-18 toolchain, and the old clang-14 bootstrap script
# was retired in the 2026-09-17 restructure. The name stays so every
# document and command written since VM-061 keeps working.
#
# apt.llvm.org publishes plain .deb files over HTTPS with no apt source
# registration needed -- `apt-get download` only searches configured
# sources, so this uses the exact same root-less "fetch the .deb, dpkg-deb
# -x it into a private prefix" recipe the old clang-14 bootstrap used, pointed at
# LLVM's own (still official, still Ubuntu-built) package host.
#
# Usage: overlume/scripts/setup_toolchain_cesium.sh
# Idempotent: does nothing (fast exit) if the prefix already has a working
# clang++.
set -euo pipefail

PREFIX="${XDG_CACHE_HOME:-$HOME/.cache}/overlume-toolchain-cesium"
DL_DIR="$PREFIX/dl"
ROOT_DIR="$PREFIX/root"

LLVM_VERSION="18"
LLVM_PKG_VERSION="18.1.8~++20240731024944+3b5b5c1ec4a3-1~exp1~20240731145000.144"
APT_LLVM_BASE="https://apt.llvm.org/jammy/pool/main/l/llvm-toolchain-${LLVM_VERSION}"

PACKAGES=(
    "clang-${LLVM_VERSION}"
    "libclang-common-${LLVM_VERSION}-dev"
    "libclang-cpp${LLVM_VERSION}"
    "libllvm${LLVM_VERSION}"
    "llvm-${LLVM_VERSION}-linker-tools"
    "libc++-${LLVM_VERSION}-dev"
    "libc++1-${LLVM_VERSION}"
    "libunwind-${LLVM_VERSION}-dev"
    "libunwind-${LLVM_VERSION}"
    "libc++abi-${LLVM_VERSION}-dev"
    "libc++abi1-${LLVM_VERSION}"
)

CLANGXX="$ROOT_DIR/usr/lib/llvm-${LLVM_VERSION}/bin/clang++"
LIBDIR1="$ROOT_DIR/usr/lib/llvm-${LLVM_VERSION}/lib"
LIBDIR2="$ROOT_DIR/usr/lib/x86_64-linux-gnu"

verify() {
    [ -x "$CLANGXX" ] || return 1
    LD_LIBRARY_PATH="$LIBDIR1:$LIBDIR2" "$CLANGXX" --version >/dev/null 2>&1 || return 1

    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' RETURN
    cat > "$tmp/probe.cxx" <<'EOF'
#include <algorithm>
#include <vector>
int main() {
    std::vector<int> v{1, 2, 3};
    std::ranges::replace(v, 1, 9);
    return v[0] == 9 ? 0 : 1;
}
EOF
    LIBRARY_PATH="$LIBDIR2" LD_LIBRARY_PATH="$LIBDIR1:$LIBDIR2" \
        "$CLANGXX" -stdlib=libc++ -std=c++20 "$tmp/probe.cxx" -o "$tmp/probe" \
            -L"$LIBDIR1" -Wl,-rpath,"$LIBDIR1" >/dev/null 2>&1 || return 1
    LD_LIBRARY_PATH="$LIBDIR1:$LIBDIR2" "$tmp/probe" || return 1
}

if verify; then
    echo "setup_toolchain_cesium.sh: clang-${LLVM_VERSION} at $CLANGXX already working, nothing to do."
    exit 0
fi

echo "setup_toolchain_cesium.sh: bootstrapping clang-${LLVM_VERSION} from apt.llvm.org into $ROOT_DIR ..."
mkdir -p "$DL_DIR" "$ROOT_DIR"
for pkg in "${PACKAGES[@]}"; do
    deb="$DL_DIR/${pkg}_${LLVM_PKG_VERSION}_amd64.deb"
    if [ ! -f "$deb" ]; then
        curl -fsSL -o "$deb" "${APT_LLVM_BASE}/${pkg}_${LLVM_PKG_VERSION}_amd64.deb"
    fi
    dpkg-deb -x "$deb" "$ROOT_DIR"
done

if verify; then
    echo "setup_toolchain_cesium.sh: OK, clang-${LLVM_VERSION} at $CLANGXX works (std::ranges::replace compiles+runs)."
else
    echo "setup_toolchain_cesium.sh: bootstrap finished but the verify probe still fails -- inspect $ROOT_DIR by hand." >&2
    exit 1
fi
