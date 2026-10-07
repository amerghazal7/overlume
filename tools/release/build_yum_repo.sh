#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# build_yum_repo.sh PKG_DIR SITE_DIR -- build SITE_DIR/rpm/<arch>/ (packages,
# repodata/repomd.xml + repomd.xml.asc) for x86_64 and aarch64 from
# PKG_DIR/*.rpm with createrepo_c, plus SITE_DIR/overlume.repo and the public
# key SITE_DIR/overlume-release.asc. Key from env (see lib_gpg.sh).
set -euo pipefail
[ $# -eq 2 ] || { echo "usage: $0 PKG_DIR SITE_DIR" >&2; exit 2; }
pkgs="$(cd "$1" && pwd)"; site="$(mkdir -p "$2" && cd "$2" && pwd)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib_gpg.sh
. "$here/lib_gpg.sh"
url="https://amerghazal7.github.io/overlume"

rm -rf "$site/rpm"
for arch in x86_64 aarch64; do
  d="$site/rpm/$arch"; mkdir -p "$d"
  n=0
  for r in "$pkgs"/*."$arch".rpm; do if [ -e "$r" ]; then cp "$r" "$d/"; n=$((n+1)); fi; done
  [ "$n" -gt 0 ] || { echo "FAIL: no $arch rpm in $pkgs"; exit 1; }
  createrepo_c --quiet "$d"
  gpg --armor --detach-sign -u "$OVERLUME_FPR" -o "$d/repodata/repomd.xml.asc" "$d/repodata/repomd.xml"
  gpg --verify "$d/repodata/repomd.xml.asc" "$d/repodata/repomd.xml" > /dev/null 2>&1 \
    || { echo "FAIL: $arch repomd.xml.asc does not verify"; exit 1; }
  echo "PASS: rpm/$arch with $n rpms"
done

gpg --armor --export "$OVERLUME_FPR" > "$site/overlume-release.asc"
cat > "$site/overlume.repo" <<EOR
[overlume]
name=Overlume
baseurl=$url/rpm/\$basearch
enabled=1
gpgcheck=1
repo_gpgcheck=1
gpgkey=$url/overlume-release.asc
EOR
echo "PASS: overlume.repo + overlume-release.asc written"
