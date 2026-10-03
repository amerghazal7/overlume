#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# publish_maven_central.sh BUNDLE_ZIP MODE -- Sonatype Central Portal publisher API.
#   MODE validate  upload (USER_MANAGED), wait for VALIDATED, then DELETE the deployment (drop).
#   MODE publish   upload (AUTOMATIC), wait for PUBLISHING/PUBLISHED.
# Credentials come from env MAVEN_CENTRAL_USERNAME / MAVEN_CENTRAL_PASSWORD; the Bearer token
# (base64 of user:password) is built in-process and handed to curl through a config file
# descriptor, never argv, never echoed. Output is HTTP codes, states and Central's `errors` JSON
# (which holds no secrets). CENTRAL_PORTAL_URL overrides the base URL (tests use a local stub);
# CENTRAL_POLL_SECONDS / CENTRAL_POLL_MAX tune polling. No `set -x` here, ever.
set -euo pipefail
bundle="${1:?usage: $0 BUNDLE_ZIP validate|publish}"
mode="${2:?usage: $0 BUNDLE_ZIP validate|publish}"
case "$mode" in validate|publish) ;; *) echo "FAIL: MODE must be validate or publish" >&2; exit 2;; esac
: "${MAVEN_CENTRAL_USERNAME:?MAVEN_CENTRAL_USERNAME is not set}"
: "${MAVEN_CENTRAL_PASSWORD:?MAVEN_CENTRAL_PASSWORD is not set}"
base="${CENTRAL_PORTAL_URL:-https://central.sonatype.com}/api/v1/publisher"
poll="${CENTRAL_POLL_SECONDS:-10}"; max="${CENTRAL_POLL_MAX:-90}"
token="$(printf '%s:%s' "$MAVEN_CENTRAL_USERNAME" "$MAVEN_CENTRAL_PASSWORD" | base64 -w0)"
name="$(basename "$bundle" .zip)"
[ "$mode" = validate ] && ptype=USER_MANAGED || ptype=AUTOMATIC

# call METHOD URL [extra curl args...] -> body in $body, HTTP code in $code
call() {
    local method="$1" url="$2"; shift 2
    local resp
    resp="$(curl -sS -X "$method" -K <(printf 'header = "Authorization: Bearer %s"\n' "$token") \
            -w '\n%{http_code}' "$@" "$url")" || { echo "FAIL: curl error on $method ${url%%\?*}"; exit 1; }
    code="${resp##*$'\n'}"; body="${resp%$'\n'*}"
}

call POST "$base/upload?publishingType=$ptype&name=$name" -F "bundle=@$bundle;type=application/zip"
echo "upload: HTTP $code"
case "$code" in 2??) ;; *) echo "FAIL: upload rejected"; exit 1;; esac
id="$body"

state=""; tries=0
while :; do
    call POST "$base/status?id=$id"
    [ "$code" = 200 ] || { echo "FAIL: status HTTP $code"; exit 1; }
    state="$(printf '%s' "$body" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("deploymentState",""))')"
    echo "status: $state"
    case "$state" in
        FAILED)
            printf '%s' "$body" | python3 -c 'import json,sys; print("errors:", json.dumps(json.load(sys.stdin).get("errors",{}), indent=2))'
            if [ "$mode" = validate ]; then call DELETE "$base/deployment/$id"; echo "drop: HTTP $code"; fi
            echo "FAIL: Central reported FAILED"; exit 1;;
        VALIDATED) [ "$mode" = validate ] && break;;
        PUBLISHING|PUBLISHED) [ "$mode" = publish ] && break;;
    esac
    tries=$((tries + 1))
    [ $tries -lt "$max" ] || { echo "FAIL: timed out in state $state"; exit 1; }
    sleep "$poll"
done

if [ "$mode" = validate ]; then
    call DELETE "$base/deployment/$id"
    echo "drop: HTTP $code"
    case "$code" in 2??) ;; *) echo "FAIL: could not drop the validated deployment"; exit 1;; esac
    echo "PASS: bundle validated by Central and dropped (nothing published)"
else
    echo "PASS: bundle accepted for publication (state $state)"
fi
