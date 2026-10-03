#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# publish_swiftpm.sh render OUT VERSION CHECKSUM URL -- write the rendered Package.swift only
# publish_swiftpm.sh check ZIP VERSION               -- compute the zip's SwiftPM checksum, render the
#     manifest for it and have `swift package dump-package` parse it (no network, nothing pushed)
# publish_swiftpm.sh push ZIP VERSION                -- the same, then commit Package.swift to
#     github.com/amerghazal7/overlume-swift and tag vVERSION, using the write deploy key in env
#     SWIFTPM_REPO_DEPLOY_KEY (throw-away ssh-agent, never written to disk or printed). Tag runs only.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
remote=git@github.com:amerghazal7/overlume-swift.git

render() {  # OUT VERSION CHECKSUM URL
    [[ "$3" =~ ^[0-9a-f]{64}$ ]] || { echo "FAIL: checksum '$3' is not 64 hex digits" >&2; exit 1; }
    sed -e "s|@CHECKSUM@|$3|g" -e "s|@URL@|$4|g" "$repo/packaging/swiftpm/Package.swift.in" > "$1"
    ! grep -q '@[A-Z0-9_]*@' "$1" || { echo "FAIL: unrendered placeholder in $1" >&2; exit 1; }
}
asset_url() { echo "https://github.com/amerghazal7/overlume/releases/download/v$1/Overlume-$1.xcframework.zip"; }

prepare() {  # ZIP VERSION OUT -> renders OUT, parses it
    local sum
    sum="$(swift package compute-checksum "$1")"
    render "$3" "$2" "$sum" "$(asset_url "$2")"
    ( cd "$(dirname "$3")" && swift package dump-package >/dev/null ) || { echo "FAIL: swift cannot parse the manifest" >&2; exit 1; }
}

case "${1:-}" in
render) shift; [ $# -eq 4 ] || { echo "usage: $0 render OUT VERSION CHECKSUM URL" >&2; exit 2; }; render "$@"; echo "PASS: rendered $1" ;;
check)
    shift; [ $# -eq 2 ] || { echo "usage: $0 check ZIP VERSION" >&2; exit 2; }
    w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
    prepare "$1" "$2" "$w/Package.swift"
    echo "PASS: Package.swift for $2 parses; checksum matches the zip"
    ;;
push)
    shift; [ $# -eq 2 ] || { echo "usage: $0 push ZIP VERSION" >&2; exit 2; }
    : "${SWIFTPM_REPO_DEPLOY_KEY:?SWIFTPM_REPO_DEPLOY_KEY is not set}"
    w="$(mktemp -d)"
    eval "$(ssh-agent -s)" >/dev/null
    trap 'ssh-agent -k >/dev/null 2>&1 || true; rm -rf "$w"' EXIT
    printf '%s\n' "$SWIFTPM_REPO_DEPLOY_KEY" | ssh-add - >/dev/null 2>&1
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=$w/known_hosts"
    git clone --quiet "$remote" "$w/repo"
    prepare "$1" "$2" "$w/repo/Package.swift"
    cd "$w/repo"
    git config user.name "overlume release"; git config user.email "release@overlume.invalid"
    git add Package.swift
    if git diff --cached --quiet; then echo "manifest unchanged"; else git commit --quiet -m "Overlume $2"; fi
    git tag "v$2" >/dev/null
    git push --quiet origin HEAD "refs/tags/v$2"
    echo "PASS: pushed Package.swift and tag v$2 to overlume-swift"
    ;;
*) echo "usage: $0 render OUT VERSION CHECKSUM URL | check ZIP VERSION | push ZIP VERSION" >&2; exit 2 ;;
esac
