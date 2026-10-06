#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_apt_repo.sh PKG_DIR SITE_DIR -- build the signed apt repository
# SITE_DIR/apt (dists/stable/{InRelease,Release,Release.gpg}, pool/) from
# every PKG_DIR/*.deb with reprepro. Key from env (see lib_gpg.sh).
set -euo pipefail
[ $# -eq 2 ] || { echo "usage: $0 PKG_DIR SITE_DIR" >&2; exit 2; }
pkgs="$(cd "$1" && pwd)"; site="$(mkdir -p "$2" && cd "$2" && pwd)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_gpg.sh
. "$here/lib_gpg.sh"

shopt -s nullglob
debs=("$pkgs"/*.deb)
[ ${#debs[@]} -gt 0 ] || { echo "FAIL: no .deb in $pkgs"; exit 1; }

base="$(mktemp -d)"
trap 'rm -rf "$base"; gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
mkdir "$base/conf"
cat > "$base/conf/distributions" <<EOD
Origin: Overlume
Label: Overlume
Codename: stable
Architectures: amd64 arm64
Components: main
Description: Overlume release packages
SignWith: $OVERLUME_FPR
EOD
for d in "${debs[@]}"; do reprepro -b "$base" -C main includedeb stable "$d" > /dev/null; done

rm -rf "$site/apt"; mkdir "$site/apt"
cp -a "$base/dists" "$base/pool" "$site/apt/"
for f in InRelease Release Release.gpg; do
  [ -s "$site/apt/dists/stable/$f" ] || { echo "FAIL: apt/dists/stable/$f missing"; exit 1; }
done
gpg --verify "$site/apt/dists/stable/InRelease" > /dev/null 2>&1 || { echo "FAIL: InRelease does not verify"; exit 1; }
echo "PASS: apt repo with ${#debs[@]} debs, signed by ${OVERLUME_FPR: -16}"
