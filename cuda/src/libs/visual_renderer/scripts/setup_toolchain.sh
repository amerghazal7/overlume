#!/usr/bin/env bash
# setup_toolchain.sh — root-less bootstrap of a clang-14/libc++-14 toolchain
# for visual_renderer.
#
# This project's dev/robot boxes have no root and no clang-14/libc++-14
# package reachable via a normal `apt install` (see plan
# docs/superpowers/plans/2026-08-18-visual-mode.md, Task 1 Step 5's fix-up
# notes). `apt-get download` fetches .debs without installing (no root
# needed) and `dpkg-deb -x` unpacks a .deb's payload into an arbitrary
# directory (also no root) -- this reuses that exact recipe and the exact
# package list that was hand-verified to work, so it's reproducible instead
# of living only in a chat transcript.
#
# Usage: cuda/src/libs/visual_renderer/scripts/setup_toolchain.sh
# Idempotent: does nothing (fast exit) if the prefix already has a clang++
# that genuinely compiles, links, and runs a static-libc++ binary.
set -euo pipefail

PREFIX="${XDG_CACHE_HOME:-$HOME/.cache}/mpviz-toolchain"
DL_DIR="$PREFIX/dl"
ROOT_DIR="$PREFIX/root"
BIN_DIR="$PREFIX/bin"

# Exact package set from Task 1 Step 5's fix-up note.
PACKAGES=(
    clang-14
    libclang-common-14-dev
    llvm-14-linker-tools
    libc++-14-dev
    libc++1-14
    libunwind-14-dev
    libunwind-14
    libc++abi-14-dev
    libc++abi1-14
    libobjc-11-dev
)

# Real compile+link+run, not just "the files exist" -- the payload is a
# fully statically-linked libc++/libc++abi/libunwind binary (see
# CMakeLists.txt for why: this is also the exact recipe visual_renderer
# itself uses, so if this passes, `cmake --toolchain ...` will too.
verify() {
    local clangxx="$BIN_DIR/clang++"
    [ -x "$clangxx" ] || return 1

    local libcxx_a lib_dir libcxxabi_a libunwind_a
    libcxx_a="$("$clangxx" -stdlib=libc++ -print-file-name=libc++.a 2>/dev/null)" || return 1
    [ -f "$libcxx_a" ] || return 1
    lib_dir="$(dirname "$libcxx_a")"
    libcxxabi_a="$lib_dir/libc++abi.a"
    libunwind_a="$lib_dir/libunwind.a"
    [ -f "$libcxxabi_a" ] || return 1
    [ -f "$libunwind_a" ] || return 1

    local tmp
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' RETURN
    cat > "$tmp/probe.cpp" <<'EOF'
#include <memory>
#include <string>
int main() {
    auto p = std::make_unique<std::string>("mpviz-toolchain-ok");
    return (*p == "mpviz-toolchain-ok") ? 0 : 1;
}
EOF
    "$clangxx" -std=c++17 -stdlib=libc++ -nostdlib++ -c "$tmp/probe.cpp" -o "$tmp/probe.o" \
        || return 1
    "$clangxx" -stdlib=libc++ -nostdlib++ "$tmp/probe.o" \
        "$libcxx_a" "$libcxxabi_a" "$libunwind_a" -o "$tmp/probe" \
        || return 1
    # No LD_LIBRARY_PATH, no toolchain on PATH: proves the binary is
    # genuinely self-sufficient, not passing only because of this shell's
    # ambient environment.
    env -i PATH=/usr/bin:/bin "$tmp/probe" || return 1
}

if verify; then
    echo "setup_toolchain.sh: $BIN_DIR/clang++ already present and verified; nothing to do."
    exit 0
fi

echo "setup_toolchain.sh: bootstrapping clang-14/libc++ into $PREFIX ..."
mkdir -p "$DL_DIR" "$ROOT_DIR" "$BIN_DIR"

for pkg in "${PACKAGES[@]}"; do
    if ! compgen -G "$DL_DIR/${pkg}_*.deb" > /dev/null; then
        echo "  apt-get download $pkg"
        (cd "$DL_DIR" && apt-get download "$pkg")
    fi
done

for deb in "$DL_DIR"/*.deb; do
    dpkg-deb -x "$deb" "$ROOT_DIR"
done

ln -sf "$ROOT_DIR/usr/bin/clang-14" "$BIN_DIR/clang"
ln -sf "$ROOT_DIR/usr/bin/clang++-14" "$BIN_DIR/clang++"

echo "setup_toolchain.sh: verifying real libc++ compile+link+run ..."
if ! verify; then
    echo "setup_toolchain.sh: verification FAILED after bootstrap." >&2
    exit 1
fi
echo "setup_toolchain.sh: OK -- $BIN_DIR/clang++ ready (static libc++/libc++abi/libunwind)."
