#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Demo preflight for the mapbox:// environment source (docs/runbooks/mapbox.md), shaped like
# overlume/scripts/check_cesium_token.sh: requests the Downtown Dubai landmark tile and one
# footprint tile inside it with $MAPBOX_TOKEN and prints one line. PASS (exit 0) when both answer
# 200, FAIL (exit 1) otherwise, SKIP (exit 2) with no request when the token is unset. It prints
# HTTP codes and curl exit statuses only, never a URL, the token or a body. Not part of the gate:
# it needs the network and a real token.

set -u
# Trim surrounding whitespace as the library does, so this checks the token the node will send.
token="${MAPBOX_TOKEN:-}"
token="${token#"${token%%[![:space:]]*}"}"
token="${token%"${token##*[![:space:]]}"}"
if [ -z "${token}" ]; then
    echo "SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)"
    exit 2
fi

# Prints "<label> HTTP <code>" or "<label> transport error (curl exit <n>)" and succeeds only on
# HTTP 200. curl reads the URL from stdin, so the token never reaches its argv (ps). No -S:
# curl's error text could carry the URL, so only its exit status is reported.
check() {
    local code rc
    code=$(printf 'url = "%s?access_token=%s"\n' "$2" "${token}" \
        | curl -s --max-time 30 --config - -o /dev/null -w '%{http_code}')
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        echo "$1 transport error (curl exit ${rc})"
        return 1
    fi
    echo "$1 HTTP ${code}"
    [ "${code}" = "200" ]
}

# Downtown Dubai: z14 10707/7006 holds landmark models; z16 42830/28025 is one of its footprint
# tiles. The endpoints are the ones the library requests.
landmarks="https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt"
footprints="https://api.mapbox.com/v4/mapbox.mapbox-streets-v8"
status=0
landmark=$(check "landmark tile" "${landmarks}/14/10707/7006.glb") || status=1
footprint=$(check "footprint tile" "${footprints}/16/42830/28025.vector.pbf") || status=1
if [ "${status}" -eq 0 ]; then
    echo "PASS: token retrieves the Downtown Dubai tiles (${landmark}, ${footprint})"
    exit 0
fi
echo "FAIL: ${landmark}, ${footprint} (see docs/runbooks/mapbox.md)"
exit 1
