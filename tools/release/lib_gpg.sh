# shellcheck shell=bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Sourced by build_apt_repo.sh / build_yum_repo.sh. Imports the release key
# from env OVERLUME_GPG_PRIVATE_KEY / OVERLUME_GPG_PASSPHRASE into a temporary
# GNUPGHOME (deleted on exit), sets non-interactive loopback signing, and
# exports GNUPGHOME and OVERLUME_FPR. Secret values are never printed.
: "${OVERLUME_GPG_PRIVATE_KEY:?OVERLUME_GPG_PRIVATE_KEY is not set}"
: "${OVERLUME_GPG_PASSPHRASE:?OVERLUME_GPG_PASSPHRASE is not set}"
GNUPGHOME="$(mktemp -d)"; export GNUPGHOME
trap 'gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$GNUPGHOME"' EXIT
chmod 700 "$GNUPGHOME"
printf '%s' "$OVERLUME_GPG_PASSPHRASE" > "$GNUPGHOME/pass"; chmod 600 "$GNUPGHOME/pass"
printf 'allow-loopback-pinentry\n' > "$GNUPGHOME/gpg-agent.conf"
printf 'batch\npinentry-mode loopback\npassphrase-file %s/pass\n' "$GNUPGHOME" > "$GNUPGHOME/gpg.conf"
printf '%s\n' "$OVERLUME_GPG_PRIVATE_KEY" | gpg --quiet --import
OVERLUME_FPR="$(gpg --list-secret-keys --with-colons | awk -F: '/^fpr:/ {print $10; exit}')"
export OVERLUME_FPR
