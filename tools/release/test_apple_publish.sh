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
sha=2d711642b726b04401627ca9fbac32f5c8530fb1903cc4db02258717921a4881
url=https://example.invalid/overlume-0.1.0-macos-universal.tar.gz

"$here/publish_homebrew.sh" render "$w/overlume.rb" 0.1.0 "$sha" "$url" >/dev/null
grep -q "^  url \"$url\"" "$w/overlume.rb" || fail "formula url not rendered"
grep -q "^  sha256 \"$sha\"" "$w/overlume.rb" || fail "formula sha256 not rendered"
grep -q '^  version "0.1.0"' "$w/overlume.rb" || fail "formula version not rendered"
! "$here/publish_homebrew.sh" render "$w/bad.rb" 0.1.0 nothex "$url" >/dev/null 2>&1 || fail "bad sha accepted"

"$here/publish_swiftpm.sh" render "$w/Package.swift" 0.1.0 "$sha" "$url" >/dev/null
grep -q "url: \"$url\", checksum: \"$sha\"" "$w/Package.swift" || fail "manifest not rendered"
! "$here/publish_swiftpm.sh" render "$w/bad.swift" 0.1.0 nothex "$url" >/dev/null 2>&1 || fail "bad checksum accepted"

# SwiftPM versions are write-once: `published` sees an existing tag, not a missing one.
git init -q --bare "$w/swift.git"; git clone -q "$w/swift.git" "$w/sw" 2>/dev/null
( cd "$w/sw" && git -c user.name=t -c user.email=t@t commit -q --allow-empty -m x && git tag v0.1.0 && git push -q origin HEAD v0.1.0 )
OVERLUME_SWIFTPM_REMOTE="file://$w/swift.git" "$here/publish_swiftpm.sh" published 0.1.0 || fail "existing tag not seen"
! OVERLUME_SWIFTPM_REMOTE="file://$w/swift.git" "$here/publish_swiftpm.sh" published 0.2.0 || fail "missing tag reported published"

# No secrets in the environment: both phases warn and exit 0.
for args in "code $w" "pkg $w/x.pkg"; do
    # shellcheck disable=SC2086
    out="$(env -u APPLE_DEVELOPER_ID_P12 -u APPLE_DEVELOPER_ID_P12_PASSWORD "$apple/sign_and_notarize.sh" $args)" \
        || fail "unsigned run of '$args' failed"
    case "$out" in *"::warning::Apple signing secrets not configured"*) ;; *) fail "'$args' did not warn" ;; esac
done
echo "PASS: Apple publish/sign scripts"
