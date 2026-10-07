#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Self-test for sign_sums.sh + merge_sums.sh with throwaway keys: merge/sign/verify round trip, then
# wrong key, tampered file (after signing and between jobs), missing signature, forged part and
# duplicate file name must all FAIL.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
t="$(mktemp -d)"; trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$t"' EXIT
export OVERLUME_GPG_PASSPHRASE=pw

mkkey() { # NAME -> $t/NAME/pub.asc, $t/NAME/sec.asc
  mkdir -p "$t/$1/h"; chmod 700 "$t/$1/h"
  GNUPGHOME="$t/$1/h" gpg --batch --quiet --pinentry-mode loopback --passphrase pw --quick-generate-key "$1@example.invalid" rsa2048 sign never
  GNUPGHOME="$t/$1/h" gpg --batch --armor --export > "$t/$1/pub.asc"
  GNUPGHOME="$t/$1/h" gpg --batch --pinentry-mode loopback --passphrase pw --armor --export-secret-keys > "$t/$1/sec.asc"
}
part() { # KEY NAME DIR
  mkdir -p "$3"; printf '%s\n' "$2" > "$3/$2.bin"
  OVERLUME_GPG_PRIVATE_KEY="$(cat "$t/$1/sec.asc")" "$here/sign_sums.sh" "$3" "$2" >/dev/null
}
expect_fail() { if "$@" >/dev/null 2>&1; then echo "FAIL: expected failure: $*"; exit 1; fi; }

mkkey good; mkkey evil
part good a "$t/parts/a"; part good b "$t/parts/b"
mkdir "$t/parts/plain"; echo z > "$t/parts/plain/z.zip"   # the one named-unsigned artifact dir: hashed fresh
"$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/parts" "$t/out" plain
OVERLUME_GPG_PRIVATE_KEY="$(cat "$t/good/sec.asc")" "$here/sign_sums.sh" "$t/out" >/dev/null
"$here/merge_sums.sh" verify "$t/good/pub.asc" "$t/out"
[ "$(wc -l < "$t/out/SHA256SUMS")" -eq 3 ] || { echo "FAIL: expected 3 lines"; exit 1; }
if grep -q 'SHA256SUMS-' "$t/out/SHA256SUMS"; then echo "FAIL: per-part sums listed in merged file"; exit 1; fi
expect_fail "$here/merge_sums.sh" verify "$t/evil/pub.asc" "$t/out"                 # wrong key

cp -r "$t/parts" "$t/p2"; echo tamper >> "$t/p2/b/b.bin"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p2" "$t/o2" plain           # tampered between jobs

cp -r "$t/parts" "$t/p3"; rm "$t/p3/a/SHA256SUMS-a.txt.asc"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p3" "$t/o3" plain           # signature missing

cp -r "$t/parts" "$t/p4"; part evil c "$t/p4/c"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p4" "$t/o4" plain           # forged part

cp -r "$t/parts" "$t/p5"; cp "$t/p5/a/a.bin" "$t/p5/plain/a.bin"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p5" "$t/o5" plain           # duplicate name

cp -r "$t/parts" "$t/p6"; echo x > "$t/p6/a/injected.bin"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p6" "$t/o6" plain           # unlisted file in signed dir

cp -r "$t/parts" "$t/p7"; rm "$t/p7/b"/SHA256SUMS-b.txt*
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p7" "$t/o7" plain           # signed dir, both sums files gone

cp -r "$t/parts" "$t/p8"; mkdir "$t/p8/pkg-stray"; echo s > "$t/p8/pkg-stray/s.bin"
expect_fail "$here/merge_sums.sh" merge "$t/good/pub.asc" "$t/p8" "$t/o8" plain           # extra unsigned directory

echo tamper >> "$t/out/a.bin"
expect_fail "$here/merge_sums.sh" verify "$t/good/pub.asc" "$t/out"                 # tampered after signing
echo "PASS: merge_sums self-test"
