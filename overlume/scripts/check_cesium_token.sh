#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -u
ASSET_ID="${1:-96188}"
if [ -z "${CESIUM_ION_TOKEN:-}" ]; then
    echo "SKIP: CESIUM_ION_TOKEN is not set (see docs/runbooks/cesium.md)"
    exit 2
fi
response=$(printf 'header = "Authorization: Bearer %s"\n' "${CESIUM_ION_TOKEN}" \
    | curl -sS --config - -w '\n%{http_code}' \
        "https://api.cesium.com/v1/assets/${ASSET_ID}/endpoint")
endpoint_code="${response##*$'\n'}"
body="${response%$'\n'*}"
if [ "${endpoint_code}" != "200" ]; then
    echo "FAIL: HTTP ${endpoint_code} from /v1/assets/${ASSET_ID}/endpoint (401=bad/expired token, 404=no access to asset)"
    exit 1
fi
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
