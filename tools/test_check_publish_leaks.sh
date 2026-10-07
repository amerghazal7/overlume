#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Self-test for check_publish_leaks.sh: a multi-entry publish-allowlist must exempt every entry.
set -euo pipefail
script="$(cd "$(dirname "$0")" && pwd)/check_publish_leaks.sh"
d="$(mktemp -d)"; trap 'rm -rf "${d}"' EXIT
cd "${d}"
h="/ho""me"  # split so this file does not itself match the path rule
git init -q . && git config user.email t@example.org && git config user.name t
git commit -q --allow-empty -m base
base="$(git rev-parse HEAD)"
printf '%s\n' '# comment' "${h}/pubA/" "${h}/pubB/" > .git/info/publish-allowlist
printf '%s\n' "${h}/pubA/x" "${h}/pubB/y" > f.txt
git add f.txt && git commit -q -m "add two public paths"
"${script}" "${base}..HEAD" >/dev/null || { echo "FAIL: allowlisted entries not exempt"; exit 1; }
echo "${h}/other/z" >> f.txt && git commit -q -am "add a private path"
out="$("${script}" "${base}..HEAD" || true)"
[[ "${out}" == *FAIL* && "${out}" == *other* && "${out}" != *pubA* && "${out}" != *pubB* ]] \
    || { echo "FAIL: expected a hit on the other path only: ${out}"; exit 1; }
echo "PASS: check_publish_leaks allowlist self-test"
