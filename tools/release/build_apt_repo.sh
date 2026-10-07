#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_apt_repo.sh PKG_DIR SITE_DIR -- build the signed apt repository
# SITE_DIR/apt (dists/stable/{InRelease,Release,Release.gpg}, pool/) from
# every PKG_DIR/*.deb with apt-ftparchive, so ALL versions are indexed
# (reprepro keeps one version per arch and cannot serve the newest N releases).
# Key from env (see lib_gpg.sh).
set -euo pipefail
[ $# -eq 2 ] || { echo "usage: $0 PKG_DIR SITE_DIR" >&2; exit 2; }
pkgs="$(cd "$1" && pwd)"; site="$(mkdir -p "$2" && cd "$2" && pwd)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_gpg.sh
. "$here/lib_gpg.sh"

shopt -s nullglob
debs=("$pkgs"/*.deb)
[ ${#debs[@]} -gt 0 ] || { echo "FAIL: no .deb in $pkgs"; exit 1; }

rm -rf "$site/apt"; mkdir -p "$site/apt/pool/main"
cp "${debs[@]}" "$site/apt/pool/main/"
cd "$site/apt"
for a in amd64 arm64; do
  d="dists/stable/main/binary-$a"; mkdir -p "$d"
  apt-ftparchive --arch "$a" packages pool > "$d/Packages"
  gzip -9nk "$d/Packages"
done
apt-ftparchive -o APT::FTPArchive::Release::Origin=Overlume -o APT::FTPArchive::Release::Label=Overlume \
  -o APT::FTPArchive::Release::Codename=stable -o APT::FTPArchive::Release::Architectures="amd64 arm64" \
  -o APT::FTPArchive::Release::Components=main release dists/stable > dists/stable/Release
gpg --default-key "$OVERLUME_FPR" --clearsign -o dists/stable/InRelease dists/stable/Release
gpg --default-key "$OVERLUME_FPR" -abs -o dists/stable/Release.gpg dists/stable/Release
cd - > /dev/null

for f in InRelease Release Release.gpg; do
  [ -s "$site/apt/dists/stable/$f" ] || { echo "FAIL: apt/dists/stable/$f missing"; exit 1; }
done
gpg --verify "$site/apt/dists/stable/InRelease" > /dev/null 2>&1 || { echo "FAIL: InRelease does not verify"; exit 1; }
gpg --verify "$site/apt/dists/stable/Release.gpg" "$site/apt/dists/stable/Release" > /dev/null 2>&1 || { echo "FAIL: Release.gpg does not verify"; exit 1; }
n="$(cat "$site"/apt/dists/stable/main/binary-*/Packages | grep -c '^Package:')"
echo "PASS: apt repo indexing $n package entries from ${#debs[@]} debs, signed by ${OVERLUME_FPR: -16}"
