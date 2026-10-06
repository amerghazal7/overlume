#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_apt_repo_versions.sh -- (1) build_apt_repo.sh must index BOTH versions of a package for
# both arches (fails if the index goes back to one version per arch); (2) the pages.yml gather
# copy (plain `cp -t`) must refuse same-named packages from two artifact dirs (fails with `cp -n`).
# Uses a throwaway key; needs gpg, dpkg-deb, apt-ftparchive.
set -euo pipefail
bad() { echo "FAIL: $*"; exit 1; }
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
t="$(mktemp -d)"; trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$t"' EXIT
mkdir -p "$t/in" "$t/key"; chmod 700 "$t/key"
for v in 0.9.0 0.10.0; do for a in amd64 arm64; do
  r="$t/root-$v-$a/DEBIAN"; mkdir -p "$r"
  printf 'Package: overlume\nVersion: %s\nArchitecture: %s\nMaintainer: t <t@example.com>\nDescription: t\n' "$v" "$a" > "$r/control"
  dpkg-deb --build "$t/root-$v-$a" "$t/in/overlume_${v}_$a.deb" > /dev/null
done; done
GNUPGHOME="$t/key" gpg --batch --quiet --pinentry-mode loopback --passphrase pw \
  --quick-generate-key "test <t@example.com>" rsa2048 sign never 2>/dev/null
export OVERLUME_GPG_PASSPHRASE=pw
OVERLUME_GPG_PRIVATE_KEY="$(GNUPGHOME="$t/key" gpg --batch --pinentry-mode loopback --passphrase pw --armor --export-secret-keys 2>/dev/null)"
export OVERLUME_GPG_PRIVATE_KEY
"$here/build_apt_repo.sh" "$t/in" "$t/site" > /dev/null || bad "build_apt_repo.sh failed"
for a in amd64 arm64; do
  p="$t/site/apt/dists/stable/main/binary-$a/Packages"
  for v in 0.9.0 0.10.0; do grep -qx "Version: $v" "$p" || bad "$a index lacks overlume $v"; done
done

# gather copy: duplicates across artifact dirs must fail, not be silently skipped
mkdir -p "$t/dl/a" "$t/dl/a-unverified" "$t/pkgs"
echo ok > "$t/dl/a/x.deb"; echo UNVERIFIED > "$t/dl/a-unverified/x.deb"
if (cd "$t" && find dl -name '*.deb' -print0 | xargs -0 cp -t pkgs) 2>/dev/null; then bad "duplicate names were not refused"; fi
echo "PASS: apt index holds both versions; gather copy refuses duplicate names"
