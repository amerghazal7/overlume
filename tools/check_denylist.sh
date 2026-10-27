#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Fails when a staged or tracked file names the deployment robot, its model, the integrating
# company or its autonomy stack: those belong to a private overlay, not this repository. The
# README's provenance sentence is the one allowed mention. Run by the lint workflow and by
# tools/hooks/pre-commit (enable with: ln -s ../../tools/hooks/pre-commit .git/hooks/pre-commit).
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

TERMS='m02p|m2[-_ ]?o1|micropolis|micropilot'
PROVENANCE='It was integrated at Micropolis Robotics on a deployed robot'

# git grep exits 1 on no match; anything else is an error, never a pass.
hits=$(git grep --cached -niE "${TERMS}" -- . ':!tools/check_denylist.sh') || [[ $? -eq 1 ]] || exit 2
# Drop the provenance sentence itself, then re-check what is left of each hit line.
hits=$(printf '%s\n' "${hits}" | sed "s/${PROVENANCE}//" | grep -iE ":.*(${TERMS})" || true)
if [[ -n "${hits}" ]]; then
    printf '%s\n' "${hits}"
    echo "FAIL: the lines above name the deployment robot or its host stack" >&2
    exit 1
fi
echo "PASS: no deny-listed name in tracked files"
