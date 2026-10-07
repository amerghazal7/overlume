#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# merge_sums.sh merge PUBKEY PARTS OUT
#   PARTS holds one directory per workflow artifact. Every SHA256SUMS-*.txt in it must verify (detached
#   .asc against PUBKEY, then sha256sum -c over its own directory); then every other file is copied
#   into OUT (a duplicate file name is an error) and OUT/SHA256SUMS is written over all of them.
#   Signing is the caller's job: sign_sums.sh OUT.
# merge_sums.sh verify PUBKEY DIR [FILE]
#   FILE (default SHA256SUMS) must carry a valid FILE.asc from PUBKEY and match the files beside it.
# Only the public key is used; no secret is read or printed.
set -euo pipefail

gpg_home() { GNUPGHOME="$(mktemp -d)"; export GNUPGHOME; chmod 700 "$GNUPGHOME"
             trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
             gpg --batch --quiet --import "$1"; }

verify() { # DIR FILE (GNUPGHOME already set)
  [ -f "$1/$2" ] && [ -f "$1/$2.asc" ] || { echo "FAIL: $1/$2(.asc) missing" >&2; return 1; }
  gpg --batch --quiet --verify "$1/$2.asc" "$1/$2" 2>/dev/null || { echo "FAIL: bad signature on $1/$2" >&2; return 1; }
  (cd "$1" && sha256sum --quiet -c "$2") || { echo "FAIL: $1/$2 does not match its files" >&2; return 1; }
}

mode="${1:-}"
case "$mode" in
merge)
  [ $# -eq 4 ] || { echo "usage: $0 merge PUBKEY PARTS OUT" >&2; exit 2; }
  pub="$2"; parts="$3"; out="$4"
  gpg_home "$pub"
  mkdir -p "$out"
  n=0
  for d in "$parts"/*/; do
    d="${d%/}"
    for s in "$d"/SHA256SUMS-*.txt; do
      [ -e "$s" ] || continue
      verify "$d" "$(basename "$s")"; n=$((n + 1))
    done
    while IFS= read -r -d '' f; do
      b="$(basename "$f")"
      case "$b" in SHA256SUMS-*) continue;; esac
      [ ! -e "$out/$b" ] || { echo "FAIL: duplicate release file name $b (in $d)" >&2; exit 1; }
      cp "$f" "$out/$b"
    done < <(find "$d" -maxdepth 1 -type f -print0)
  done
  [ "$n" -gt 0 ] || { echo "FAIL: no SHA256SUMS-*.txt found under $parts" >&2; exit 1; }
  (cd "$out" && find . -maxdepth 1 -type f ! -name 'SHA256SUMS*' -printf '%f\n' | LC_ALL=C sort | xargs -r sha256sum > SHA256SUMS)
  echo "PASS: merged $n verified checksum files, $(wc -l < "$out/SHA256SUMS") files in $out/SHA256SUMS" ;;
verify)
  [ $# -ge 3 ] && [ $# -le 4 ] || { echo "usage: $0 verify PUBKEY DIR [FILE]" >&2; exit 2; }
  gpg_home "$2"
  verify "$3" "${4:-SHA256SUMS}"
  echo "PASS: ${4:-SHA256SUMS} verifies against $2" ;;
*) echo "usage: $0 merge PUBKEY PARTS OUT | verify PUBKEY DIR [FILE]" >&2; exit 2 ;;
esac
