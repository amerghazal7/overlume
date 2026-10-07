#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Apple signing, in the two phases the packaging order needs:
#   sign_and_notarize.sh code PATH...   codesign (hardened runtime, timestamp) each Mach-O under PATH
#   sign_and_notarize.sh pkg FILE.pkg   productsign, notarytool submit --wait, stapler staple
# Runs only when the secrets exist (names; values are never printed, no set -x):
#   APPLE_DEVELOPER_ID_P12 (base64 .p12 holding the Developer ID Application + Installer
#   certificates), APPLE_DEVELOPER_ID_P12_PASSWORD, and for notarisation APPLE_NOTARY_KEY_ID,
#   APPLE_NOTARY_ISSUER_ID, APPLE_NOTARY_KEY_P8 (the API key's contents).
# Without them each phase emits ::warning:: and exits 0 (packages stay unsigned) -- never silent.
set -euo pipefail

phase="${1:-}"; shift || true
[ -n "$phase" ] && [ $# -ge 1 ] || { echo "usage: $0 code PATH... | pkg FILE.pkg" >&2; exit 2; }

if [ -z "${APPLE_DEVELOPER_ID_P12:-}" ] || [ -z "${APPLE_DEVELOPER_ID_P12_PASSWORD:-}" ]; then
    echo "::warning::Apple signing secrets not configured; $phase phase skipped, packages unsigned"
    exit 0
fi

work="$(mktemp -d)"; chmod 700 "$work"
keychain="$work/overlume.keychain-db"
orig_keychains="$(security list-keychains -d user | tr -d '"' | tr '\n' ' ')"
cleanup() {
    # shellcheck disable=SC2086
    security list-keychains -d user -s $orig_keychains >/dev/null 2>&1 || true
    security delete-keychain "$keychain" >/dev/null 2>&1 || true
    rm -rf "$work"
}
trap cleanup EXIT

kc_pass="$(openssl rand -hex 16)"
security create-keychain -p "$kc_pass" "$keychain"
security set-keychain-settings -lut 3600 "$keychain"
security unlock-keychain -p "$kc_pass" "$keychain"
printf '%s' "$APPLE_DEVELOPER_ID_P12" | base64 --decode > "$work/id.p12"
security import "$work/id.p12" -k "$keychain" -P "$APPLE_DEVELOPER_ID_P12_PASSWORD" \
    -T /usr/bin/codesign -T /usr/bin/productsign >/dev/null
rm -f "$work/id.p12"
security set-key-partition-list -S apple-tool:,apple: -s -k "$kc_pass" "$keychain" >/dev/null
# shellcheck disable=SC2086
security list-keychains -d user -s "$keychain" $orig_keychains

identity() {  # "Developer ID Application" | "Developer ID Installer" -> SHA-1 of the first match
    security find-identity -v "$keychain" | awk -v want="$1" 'index($0, want) {print $2; exit}'
}

case "$phase" in
code)
    app="$(identity "Developer ID Application")"
    [ -n "$app" ] || { echo "::warning::no Developer ID Application identity in APPLE_DEVELOPER_ID_P12; unsigned"; exit 0; }
    n=0
    while IFS= read -r f; do
        file -b "$f" | grep -q 'Mach-O' || continue
        codesign --force --timestamp --options runtime --sign "$app" --keychain "$keychain" "$f"
        n=$((n + 1))
    done < <(find "$@" -type f)
    echo "PASS: codesigned $n Mach-O file(s)"
    ;;
pkg)
    pkg="$1"
    inst="$(identity "Developer ID Installer")"
    [ -n "$inst" ] || { echo "::warning::no Developer ID Installer identity in APPLE_DEVELOPER_ID_P12; pkg unsigned"; exit 0; }
    productsign --sign "$inst" --keychain "$keychain" "$pkg" "$work/signed.pkg"
    mv "$work/signed.pkg" "$pkg"
    pkgutil --check-signature "$pkg" >/dev/null
    echo "PASS: $(basename "$pkg") signed"
    if [ -z "${APPLE_NOTARY_KEY_ID:-}" ] || [ -z "${APPLE_NOTARY_ISSUER_ID:-}" ] || [ -z "${APPLE_NOTARY_KEY_P8:-}" ]; then
        echo "::warning::Apple notary secrets not configured; $(basename "$pkg") signed but not notarised"
        exit 0
    fi
    (umask 077; printf '%s\n' "$APPLE_NOTARY_KEY_P8" > "$work/key.p8")
    xcrun notarytool submit "$pkg" --key "$work/key.p8" --key-id "$APPLE_NOTARY_KEY_ID" \
        --issuer "$APPLE_NOTARY_ISSUER_ID" --wait
    xcrun stapler staple "$pkg"
    echo "PASS: $(basename "$pkg") notarised and stapled"
    ;;
*)
    echo "usage: $0 code PATH... | pkg FILE.pkg" >&2; exit 2 ;;
esac
