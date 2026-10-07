#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Pre-merge / pre-push check: nothing local or internal reaches the public origin.
# Scans the lines ADDED in a commit range and the range's commit messages for
#   - local absolute paths (home directories, agent scratch/project directories), and
#   - the internal names listed, one extended regex per line, in the UNTRACKED file
#     $(git rev-parse --git-common-dir)/info/publish-denylist (never commit that list:
#     naming the internal repos here would publish them).
# Usage: tools/check_publish_leaks.sh [RANGE]   (default origin/main..HEAD)
# Prints PASS, or FAIL with commit/file/line for each hit; exits 1 on any hit.
set -euo pipefail

range="${1:-origin/main..HEAD}"
denylist="$(git rev-parse --git-common-dir)/info/publish-denylist"

patterns=(
    '/home/[A-Za-z0-9._-]+/'
    '/Users/[A-Za-z0-9._-]+/'
    '/tmp/claude-[0-9]+'
    '[.]claude/projects/'  # bracketed so this line cannot match itself
)
if [[ -f "${denylist}" ]]; then
    while IFS= read -r line; do
        [[ -z "${line}" || "${line}" == \#* ]] && continue
        patterns+=("${line}")
    done < "${denylist}"
else
    echo "WARN  no ${denylist}: only generic path patterns checked"
fi
regex="$(IFS='|'; echo "${patterns[*]}")"

hits="$(git log -p --no-color --format='@@commit %h' "${range}" -- |
    awk -v re="${regex}" '
        /^@@commit / { c = $2; next }
        /^\+\+\+ /   { f = substr($0, 7); next }
        /^\+/ && $0 ~ re { printf "  %s %s: %s\n", c, f, substr($0, 2, 160) }')"
msg_hits="$(git log --no-color --format='@@commit %h%n%B' "${range}" |
    awk -v re="${regex}" '
        /^@@commit / { c = $2; next }
        $0 ~ re { printf "  %s (commit message): %s\n", c, substr($0, 1, 160) }')"

n_commits="$(git rev-list --count "${range}")"
if [[ -n "${hits}${msg_hits}" ]]; then
    echo "FAIL  publish leak check over ${range} (${n_commits} commits):"
    [[ -n "${hits}" ]] && echo "${hits}"
    [[ -n "${msg_hits}" ]] && echo "${msg_hits}"
    echo "Scrub before publishing: rewrite the unpushed commits with placeholders, never push and fix after."
    exit 1
fi
echo "PASS  publish leak check over ${range} (${n_commits} commits)"
