#!/usr/bin/env bash
# setup_toolchain_cesium.sh -- root-less bootstrap of a NEWER clang/libc++
# toolchain used ONLY to build cesium-native + its vcpkg ports (VM-061).
#
# WHY A SECOND TOOLCHAIN (recorded deviation, verified empirically at
# implementation): this project's primary toolchain (setup_toolchain.sh,
# clang-14/libc++-14) is enough for visual_renderer's own C++17 code, but
# cesium-native's vcpkg dependency "ada-url" hard-requires C++20 and its
# url_search_params-inl.h calls `std::ranges::replace` -- a ranges
# <algorithm> overload libc++-14 (2022-era) does not yet implement. Verified
# directly: a trivial `std::ranges::replace` program fails to compile under
# both libc++-14 AND libc++-15 (also apt-installable on this box) with the
# identical "no member named 'replace' in namespace 'std::ranges'" error,
# but compiles and runs under libc++-18. clang-16/17 are not packaged in
# Ubuntu 22.04's own apt repos (checked: no candidate), so this reaches past
# them straight to apt.llvm.org's jammy channel (LLVM's own official binary
# repo) for clang-18 specifically -- the plan's own named fallback for
# exactly this class of gap (Decision 15.2c: "a newer local clang via
# scripts/setup_toolchain.sh"). This is ONLY used for the vcpkg overlay
# triplet's chainload toolchain (cmake/vcpkg-clang-libcxx-toolchain.cmake)
# -- visual_renderer's own primary toolchain (toolchain-clang-libcxx.cmake)
# is UNCHANGED, still clang-14, so none of visual_renderer's own C++17
# translation units, yaml-cpp, or GoogleTest are affected. libc++'s ABI is
# stable across LLVM versions for the default (non-"unstable") ABI
# configuration both distro packages use (`std::__1::`, unchanged struct
# layouts) -- this is the same property that lets a distro ship multiple
# clang/libc++ versions side by side at all -- so archives built here can
# still be `ld -r`-merged with visual_renderer's own clang-14 objects
# (scripts/merge_yamlcpp.sh); Task 2 Step 1's strings/nm ABI check and
# Step 6's full node + gtest rebuild are the empirical proof this holds in
# practice, not just in theory.
#
# apt.llvm.org publishes plain .deb files over HTTPS with no apt source
# registration needed -- `apt-get download` only searches configured
# sources, so this uses the exact same root-less "fetch the .deb, dpkg-deb
# -x it into a private prefix" recipe as setup_toolchain.sh, just pointed at
# a different (still official, still Ubuntu-built) package host.
#
# Usage: cuda/src/libs/visual_renderer/scripts/setup_toolchain_cesium.sh
# Idempotent: does nothing (fast exit) if the prefix already has a working
# clang++.
set -euo pipefail

PREFIX="${XDG_CACHE_HOME:-$HOME/.cache}/mpviz-toolchain-cesium"
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
