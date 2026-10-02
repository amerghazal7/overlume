#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# sign_sums.sh DIR NAME -- write DIR/SHA256SUMS-NAME.txt (every other file in
# DIR) plus a detached armored signature DIR/SHA256SUMS-NAME.txt.asc.
# GPG material comes from env OVERLUME_GPG_PRIVATE_KEY (armored) and
# OVERLUME_GPG_PASSPHRASE; it is imported into a temporary GNUPGHOME that is
# deleted on exit. Secret values are never printed.
set -euo pipefail

if [ $# -ne 2 ]; then echo "usage: $0 DIR NAME" >&2; exit 2; fi
dir="$1"; name="$2"
: "${OVERLUME_GPG_PRIVATE_KEY:?OVERLUME_GPG_PRIVATE_KEY is not set}"
: "${OVERLUME_GPG_PASSPHRASE:?OVERLUME_GPG_PASSPHRASE is not set}"

sums="SHA256SUMS-${name}.txt"
(cd "$dir" && find . -maxdepth 1 -type f ! -name 'SHA256SUMS-*' -printf '%f\n' | LC_ALL=C sort \
   | xargs -r sha256sum > "$sums")

GNUPGHOME="$(mktemp -d)"; export GNUPGHOME
trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
chmod 700 "$GNUPGHOME"
printf '%s\n' "$OVERLUME_GPG_PRIVATE_KEY" | gpg --batch --quiet --import
fpr="$(gpg --batch --list-secret-keys --with-colons | awk -F: '/^fpr:/ {print $10; exit}')"
printf '%s' "$OVERLUME_GPG_PASSPHRASE" | gpg --batch --yes --quiet --pinentry-mode loopback \
    --passphrase-fd 0 --armor --detach-sign -u "$fpr" -o "$dir/$sums.asc" "$dir/$sums"
gpg --batch --quiet --verify "$dir/$sums.asc" "$dir/$sums"
echo "PASS: $sums signed (key ${fpr: -16})"
