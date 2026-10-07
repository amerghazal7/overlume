#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_apt_repo_versions.sh -- (1) build_apt_repo.sh must index BOTH versions of a package for
# both arches (fails if the index goes back to one version per arch); (2) pages.yml's OWN gather
# lines (extracted from the workflow) must skip *-unverified* artifacts and refuse same-named
# packages from two artifact dirs (fails if the workflow drifts to `cp -n` or loses the prune).
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

# gather copy: run pages.yml's own find|xargs lines (fails if they drift to cp -n or lose the prune)
gather="$(sed -n '/find dl -path/,/xargs -0 cp/p' "$here/../../.github/workflows/pages.yml")"
[ -n "$gather" ] || bad "gather copy not found in pages.yml"
mkdir -p "$t/g1/dl/a" "$t/g1/dl/a-unverified" "$t/g1/pkgs" "$t/g2/dl/a" "$t/g2/dl/b" "$t/g2/pkgs"
echo ok > "$t/g1/dl/a/x.deb"; echo UNVERIFIED > "$t/g1/dl/a-unverified/x.deb"
(cd "$t/g1" && bash -euo pipefail -c "$gather") || bad "gather copy failed on verified + unverified"
grep -qx ok "$t/g1/pkgs/x.deb" || bad "gather copy took the unverified package"
echo a > "$t/g2/dl/a/x.deb"; echo b > "$t/g2/dl/b/x.deb"
if (cd "$t/g2" && bash -euo pipefail -c "$gather") 2>/dev/null; then bad "duplicate names were not refused"; fi
echo "PASS: apt index holds both versions; pages.yml gather skips unverified and refuses duplicate names"
