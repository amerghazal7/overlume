#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# sign_rpms.sh DIR -- `rpmsign --addsign` every DIR/*.rpm with the release key
# (env OVERLUME_GPG_PRIVATE_KEY / OVERLUME_GPG_PASSPHRASE), then check each
# with `rpm -K` against the same key. Needs rpmsign (package rpm-sign).
# Secret values are never printed; the temp GNUPGHOME is deleted on exit.
set -euo pipefail

if [ $# -ne 1 ]; then echo "usage: $0 DIR" >&2; exit 2; fi
dir="$1"
: "${OVERLUME_GPG_PRIVATE_KEY:?OVERLUME_GPG_PRIVATE_KEY is not set}"
: "${OVERLUME_GPG_PASSPHRASE:?OVERLUME_GPG_PASSPHRASE is not set}"

GNUPGHOME="$(mktemp -d)"; export GNUPGHOME
trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
chmod 700 "$GNUPGHOME"
printf '%s\n' "$OVERLUME_GPG_PRIVATE_KEY" | gpg --batch --quiet --import
fpr="$(gpg --batch --list-secret-keys --with-colons | awk -F: '/^fpr:/ {print $10; exit}')"
printf '%s' "$OVERLUME_GPG_PASSPHRASE" > "$GNUPGHOME/pass"; chmod 600 "$GNUPGHOME/pass"

shopt -s nullglob
rpms=("$dir"/*.rpm)
[ ${#rpms[@]} -gt 0 ] || { echo "FAIL: no rpm in $dir"; exit 1; }
rpmsign --define "_gpg_name $fpr" \
        --define "__gpg_sign_cmd %{__gpg} gpg --no-verbose --no-armor --batch --pinentry-mode loopback --passphrase-file $GNUPGHOME/pass --no-secmem-warning -u \"%{_gpg_name}\" -sbo %{__signature_filename} --digest-algo sha256 %{__plaintext_filename}" \
        --addsign "${rpms[@]}" > /dev/null

# Verify against a throwaway rpmdb holding only the public key.
gpg --batch --armor --export "$fpr" > "$GNUPGHOME/pub.asc"
mkdir "$GNUPGHOME/rpmdb"
rpm --dbpath "$GNUPGHOME/rpmdb" --import "$GNUPGHOME/pub.asc"
for r in "${rpms[@]}"; do
    out="$(rpm --dbpath "$GNUPGHOME/rpmdb" -K "$r")"
    case "$out" in
        *"digests signatures OK"*) echo "PASS: $(basename "$r") signed";;
        *) echo "FAIL: $(basename "$r"): $out"; exit 1;;
    esac
done
