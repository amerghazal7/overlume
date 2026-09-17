#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# Smoke check (VM-060): does CESIUM_ION_TOKEN retrieve the asset's tileset.json?
# Usage: check_cesium_token.sh [asset_id]   (default 96188, Cesium OSM Buildings)
# Two stages: (1) the /v1/assets/<id>/endpoint handshake, (2) fetching the
# tileset.json URL that handshake returned, with its short-lived session token.
# PASS requires BOTH to be HTTP 200 -- an endpoint 200 alone only proves
# assets:read, not the AC ("Token retrieves tileset.json").
# Prints PASS/FAIL only. NEVER prints the token, the session token, or any
# response body -- the endpoint body embeds a short-lived session accessToken.
set -u  # deliberately no -x, ever, in this file
ASSET_ID="${1:-96188}"
if [ -z "${CESIUM_ION_TOKEN:-}" ]; then
    echo "SKIP: CESIUM_ION_TOKEN is not set (see docs/runbooks/cesium.md)"
    exit 2
fi
# Stage 1: the endpoint handshake. The token reaches curl via --config on
# stdin, NOT argv: argv is world-readable through /proc on a shared box for
# the request's lifetime. The body lands in a shell variable, never on stdout.
response=$(printf 'header = "Authorization: Bearer %s"\n' "${CESIUM_ION_TOKEN}" \
    | curl -sS --config - -w '\n%{http_code}' \
        "https://api.cesium.com/v1/assets/${ASSET_ID}/endpoint")
endpoint_code="${response##*$'\n'}"
body="${response%$'\n'*}"
if [ "${endpoint_code}" != "200" ]; then
    echo "FAIL: HTTP ${endpoint_code} from /v1/assets/${ASSET_ID}/endpoint (401=bad/expired token, 404=no access to asset)"
    exit 1
fi
# Stage 2: pull url + accessToken out of the body (no jq dependency; ion's
# endpoint JSON is flat for these two string fields) and fetch tileset.json
# with the SESSION token -- same stdin --config argv-avoidance as stage 1.
tileset_url=$(printf '%s' "${body}" | grep -o '"url"[[:space:]]*:[[:space:]]*"[^"]*"' | head -1 | sed 's/.*"\([^"]*\)"$/\1/')
session_token=$(printf '%s' "${body}" | grep -o '"accessToken"[[:space:]]*:[[:space:]]*"[^"]*"' | head -1 | sed 's/.*"\([^"]*\)"$/\1/')
if [ -z "${tileset_url}" ] || [ -z "${session_token}" ]; then
    echo "FAIL: endpoint 200 but could not parse url/accessToken from the response (body not printed by design)"
    exit 1
fi
tileset_code=$(printf 'header = "Authorization: Bearer %s"\n' "${session_token}" \
    | curl -sS --config - -o /dev/null -w '%{http_code}' "${tileset_url}")
if [ "${tileset_code}" = "200" ]; then
    echo "PASS: token retrieves tileset.json for asset ${ASSET_ID} (endpoint 200, tileset 200)"
    exit 0
fi
echo "FAIL: endpoint 200 but tileset.json fetch returned HTTP ${tileset_code} (session token accepted by the endpoint, rejected or unroutable at the tileset URL)"
exit 1
