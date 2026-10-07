#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# merge_sums.sh merge PUBKEY PARTS OUT [UNSIGNED_DIR...]
#   PARTS holds one directory per workflow artifact. Every directory except the named UNSIGNED_DIRs must hold
#   exactly one SHA256SUMS-*.txt that verifies (detached .asc against PUBKEY, sha256sum -c) and whose listed
#   names equal the directory's other files (nothing extra, nothing missing). Files are then copied into OUT
#   (a duplicate file name is an error) and OUT/SHA256SUMS is written over all of them.
#   Signing is the caller's job: sign_sums.sh OUT.
# merge_sums.sh verify PUBKEY DIR [FILE]
#   FILE (default SHA256SUMS) must carry a valid FILE.asc from PUBKEY and match the files beside it.
# Only the public key is used; no secret is read or printed.
set -euo pipefail

gpg_home() { GNUPGHOME="$(mktemp -d)"; export GNUPGHOME; chmod 700 "$GNUPGHOME"
             trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
             gpg --batch --quiet --import "$1"; }

verify() { # DIR FILE (GNUPGHOME already set)
  if ! { [ -f "$1/$2" ] && [ -f "$1/$2.asc" ]; }; then echo "FAIL: $1/$2(.asc) missing" >&2; return 1; fi
  gpg --batch --quiet --verify "$1/$2.asc" "$1/$2" 2>/dev/null || { echo "FAIL: bad signature on $1/$2" >&2; return 1; }
  (cd "$1" && sha256sum --quiet -c "$2") || { echo "FAIL: $1/$2 does not match its files" >&2; return 1; }
}

mode="${1:-}"
case "$mode" in
merge)
  [ $# -ge 4 ] || { echo "usage: $0 merge PUBKEY PARTS OUT [UNSIGNED_DIR...]" >&2; exit 2; }
  pub="$2"; parts="$3"; out="$4"; shift 4; unsigned=" $* "
  gpg_home "$pub"
  mkdir -p "$out"
  n=0
  for d in "$parts"/*/; do
    d="${d%/}"
    if [[ "$unsigned" != *" $(basename "$d") "* ]]; then
      sums=("$d"/SHA256SUMS-*.txt)
      if ! { [ "${#sums[@]}" -eq 1 ] && [ -e "${sums[0]}" ]; }; then echo "FAIL: $d must hold exactly one SHA256SUMS-*.txt" >&2; exit 1; fi
      verify "$d" "$(basename "${sums[0]}")"; n=$((n + 1))
      extra="$(comm -3 <(awk '{print $2}' "${sums[0]}" | LC_ALL=C sort) \
        <(find "$d" -maxdepth 1 -type f ! -name 'SHA256SUMS-*' -printf '%f\n' | LC_ALL=C sort))"
      [ -z "$extra" ] || { echo "FAIL: $d files differ from its checksum list: $extra" >&2; exit 1; }
    fi
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
  if ! { [ $# -ge 3 ] && [ $# -le 4 ]; }; then echo "usage: $0 verify PUBKEY DIR [FILE]" >&2; exit 2; fi
  gpg_home "$2"
  verify "$3" "${4:-SHA256SUMS}"
  echo "PASS: ${4:-SHA256SUMS} verifies against $2" ;;
*) echo "usage: $0 merge PUBKEY PARTS OUT | verify PUBKEY DIR [FILE]" >&2; exit 2 ;;
esac
