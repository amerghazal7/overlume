#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_maven_publish.sh [AAR] -- build a signed bundle with a throwaway GPG key and drive
# publish_maven_central.sh against the local Central Portal stub: validate+drop, publish,
# Central-reported FAILED, bad credentials, and "no secret in the output". Fake credentials only.
# With no AAR a tiny stand-in zip is used (the bundle layout, not the AAR, is under test).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
work="$(mktemp -d)"; srvpid=""
trap '[ -n "$srvpid" ] && kill "$srvpid" 2>/dev/null; gpgconf --kill gpg-agent >/dev/null 2>&1 || true; rm -rf "$work"' EXIT
fail() { echo "FAIL: $*"; exit 1; }

aar="${1:-}"
if [ -z "$aar" ]; then
    mkdir -p "$work/aar"; echo x > "$work/aar/AndroidManifest.xml"
    (cd "$work/aar" && zip -qr overlume-9.9.9-android.aar .); aar="$work/aar/overlume-9.9.9-android.aar"
fi

# Throwaway signing key; the real one exists only as a GitHub Actions secret.
export GNUPGHOME="$work/gnupg"; mkdir -m 700 "$GNUPGHOME"
export OVERLUME_GPG_PASSPHRASE="test-passphrase-not-real"
gpg --batch --quiet --pinentry-mode loopback --passphrase "$OVERLUME_GPG_PASSPHRASE" \
    --quick-generate-key "Overlume Test <test@example.invalid>" rsa2048 sign never
OVERLUME_GPG_PRIVATE_KEY="$(gpg --batch --pinentry-mode loopback --passphrase "$OVERLUME_GPG_PASSPHRASE" \
    --armor --export-secret-keys)"
export OVERLUME_GPG_PRIVATE_KEY
bundle="$work/overlume-bundle.zip"
GNUPGHOME="$work/gnupg-build" "$here/build_maven_bundle.sh" "$aar" "$bundle" || fail "bundle build"
n="$(unzip -Z1 "$bundle" | grep -vc '/$')"; [ "$n" -eq 24 ] || fail "bundle has $n entries, want 24 (4 artifacts, each + .asc, each with .md5 and .sha1)"
unzip -Z1 "$bundle" | grep -q '^io/github/amerghazal7/overlume/.*/overlume-.*\.pom\.asc$' || fail "pom.asc missing"

export STUB_USER="stub-user" STUB_PASSWORD="stub-pass-s3cret-value"
token="$(printf '%s:%s' "$STUB_USER" "$STUB_PASSWORD" | base64 -w0)"
export MAVEN_CENTRAL_USERNAME="$STUB_USER" MAVEN_CENTRAL_PASSWORD="$STUB_PASSWORD"
export CENTRAL_POLL_SECONDS=0 CENTRAL_POLL_MAX=10

run_stub() {  # scenario
    : > "$work/requests.log"; rm -f "$work/port"
    STUB_SCENARIO="$1" python3 "$here/stub_central_portal.py" "$work/port" "$work/requests.log" &
    srvpid=$!
    until [ -s "$work/port" ]; do sleep 0.1; done
    export CENTRAL_PORTAL_URL="http://127.0.0.1:$(cat "$work/port")"
}
stop_stub() { kill "$srvpid"; wait "$srvpid" 2>/dev/null || true; srvpid=""; }
no_leak() { ! grep -qF -e "$STUB_PASSWORD" -e "$token" "$1" || fail "credential material in output ($1)"; }

# 1. validate: VALIDATED then dropped
run_stub ok
"$here/publish_maven_central.sh" "$bundle" validate > "$work/o1" 2>&1 || { cat "$work/o1"; fail "validate flow"; }
grep -q 'status: VALIDATED' "$work/o1" && grep -q '^PASS' "$work/o1" || fail "validate output"
grep -q '^DELETE /api/v1/publisher/deployment/.* HTTP 204' "$work/requests.log" || fail "validate did not drop the deployment"
no_leak "$work/o1"; stop_stub

# 2. publish: AUTOMATIC, no DELETE
run_stub ok
"$here/publish_maven_central.sh" "$bundle" publish > "$work/o2" 2>&1 || { cat "$work/o2"; fail "publish flow"; }
grep -q 'upload: HTTP 201' "$work/o2" && grep -qE 'status: PUBLISH(ING|ED)' "$work/o2" || fail "publish output"
! grep -q '^DELETE' "$work/requests.log" || fail "publish must not drop"
no_leak "$work/o2"; stop_stub

# 3. Central says FAILED: errors printed, exit 1, deployment dropped
run_stub reject
if "$here/publish_maven_central.sh" "$bundle" validate > "$work/o3" 2>&1; then fail "FAILED state must exit non-zero"; fi
grep -q 'Invalid signature' "$work/o3" || fail "errors JSON not printed"
grep -q '^DELETE .* HTTP 204' "$work/requests.log" || fail "FAILED deployment not dropped"
no_leak "$work/o3"; stop_stub

# 4. bad credentials: HTTP 401 only
run_stub ok
if MAVEN_CENTRAL_PASSWORD=wrong "$here/publish_maven_central.sh" "$bundle" validate > "$work/o4" 2>&1; then fail "401 must exit non-zero"; fi
grep -q 'upload: HTTP 401' "$work/o4" || fail "401 not reported"
no_leak "$work/o4"; stop_stub

# 5. a bundle missing a signature is rejected by the stub's Central-style checks
mkdir "$work/x"; (cd "$work/x" && unzip -q "$bundle" && find . -name 'overlume-*.pom.asc' -delete && zip -qr "$work/unsigned.zip" io)
run_stub ok
if "$here/publish_maven_central.sh" "$work/unsigned.zip" validate > "$work/o5" 2>&1; then fail "unsigned bundle must fail"; fi
grep -q 'missing .asc' "$work/o5" || fail "missing-signature error not shown"
stop_stub
echo "PASS: maven publish script (validate+drop, publish, FAILED, 401, unsigned bundle)"
