#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Host-independent checks of the Apple release scripts: the Homebrew formula and SwiftPM manifest
# render with every placeholder filled and reject a malformed digest; sign_and_notarize.sh without
# its secrets warns (never silent) and succeeds for both phases.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
apple="$here/../apple"
w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
fail() { echo "FAIL: $*" >&2; exit 1; }
sha=$(printf 'x' | sha256sum | cut -d' ' -f1)
url=https://example.invalid/overlume-0.1.0-macos-universal.tar.gz

"$here/publish_homebrew.sh" render "$w/overlume.rb" 0.1.0 "$sha" "$url" >/dev/null
grep -q "^  url \"$url\"" "$w/overlume.rb" || fail "formula url not rendered"
grep -q "^  sha256 \"$sha\"" "$w/overlume.rb" || fail "formula sha256 not rendered"
grep -q '^  version "0.1.0"' "$w/overlume.rb" || fail "formula version not rendered"
! "$here/publish_homebrew.sh" render "$w/bad.rb" 0.1.0 nothex "$url" >/dev/null 2>&1 || fail "bad sha accepted"

"$here/publish_swiftpm.sh" render "$w/Package.swift" 0.1.0 "$sha" "$url" >/dev/null
grep -q "url: \"$url\", checksum: \"$sha\"" "$w/Package.swift" || fail "manifest not rendered"
! "$here/publish_swiftpm.sh" render "$w/bad.swift" 0.1.0 nothex "$url" >/dev/null 2>&1 || fail "bad checksum accepted"

# No secrets in the environment: both phases warn and exit 0.
for args in "code $w" "pkg $w/x.pkg"; do
    # shellcheck disable=SC2086
    out="$(env -u APPLE_DEVELOPER_ID_P12 -u APPLE_DEVELOPER_ID_P12_PASSWORD "$apple/sign_and_notarize.sh" $args)" \
        || fail "unsigned run of '$args' failed"
    case "$out" in *"::warning::Apple signing secrets not configured"*) ;; *) fail "'$args' did not warn" ;; esac
done
echo "PASS: Apple publish/sign scripts"
