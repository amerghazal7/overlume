#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# publish_homebrew.sh render OUT.rb VERSION SHA256 URL   -- write the rendered formula only
# publish_homebrew.sh push VERSION SHA256 URL            -- render and push Formula/overlume.rb to
#     github.com/amerghazal7/homebrew-overlume with the write deploy key in env HOMEBREW_TAP_DEPLOY_KEY
#     (loaded into a throw-away ssh-agent; never written to disk or printed). Tag runs only.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
tap=git@github.com:amerghazal7/homebrew-overlume.git

render() {  # OUT VERSION SHA256 URL
    [[ "$3" =~ ^[0-9a-f]{64}$ ]] || { echo "FAIL: sha256 '$3' is not 64 hex digits" >&2; exit 1; }
    sed -e "s|@VERSION@|$2|g" -e "s|@SHA256@|$3|g" -e "s|@URL@|$4|g" \
        "$repo/packaging/homebrew/overlume.rb.in" > "$1"
    ! grep -q '@[A-Z0-9_]*@' "$1" || { echo "FAIL: unrendered placeholder in $1" >&2; exit 1; }
}

case "${1:-}" in
render) shift; [ $# -eq 4 ] || { echo "usage: $0 render OUT.rb VERSION SHA256 URL" >&2; exit 2; }; render "$@"; echo "PASS: rendered $1" ;;
push)
    shift; [ $# -eq 3 ] || { echo "usage: $0 push VERSION SHA256 URL" >&2; exit 2; }
    : "${HOMEBREW_TAP_DEPLOY_KEY:?HOMEBREW_TAP_DEPLOY_KEY is not set}"
    w="$(mktemp -d)"
    eval "$(ssh-agent -s)" >/dev/null
    trap 'ssh-agent -k >/dev/null 2>&1 || true; rm -rf "$w"' EXIT
    printf '%s\n' "$HOMEBREW_TAP_DEPLOY_KEY" | ssh-add - >/dev/null 2>&1
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=$w/known_hosts"
    git clone --quiet "$tap" "$w/tap"
    mkdir -p "$w/tap/Formula"
    render "$w/tap/Formula/overlume.rb" "$1" "$2" "$3"
    cd "$w/tap"
    git config user.name "overlume release"; git config user.email "release@overlume.invalid"
    git add Formula/overlume.rb
    if git diff --cached --quiet; then echo "PASS: tap already at $1"; exit 0; fi
    git commit --quiet -m "overlume $1"
    git push --quiet origin HEAD
    echo "PASS: pushed overlume $1 to the tap"
    ;;
*) echo "usage: $0 render OUT.rb VERSION SHA256 URL | push VERSION SHA256 URL" >&2; exit 2 ;;
esac
