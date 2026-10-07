# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# shellcheck shell=bash
#
# Sourced by tools/validate_visual_mode.sh. A rig whose vcam WS bridge failed to bind used to
# keep running with the node and stream up and no bridge (the failure was only in the bridge
# log), which reads as "vcam orbit is broken". These two checks make that fail loudly instead.

# The listening socket on TCP port $1, with its owner when visible ("" when nothing listens).
port_holder() {
    ss -ltnpH "sport = :$1" 2>/dev/null
}

# Fail when TCP port $1 is already taken; $2 names what needs it. Run AFTER kill_prior_rig, so a
# hit is a foreign process (another tool's server), never this rig's own leftovers.
require_port_free() {
    if ! command -v ss >/dev/null 2>&1; then
        echo "[preflight] ss not found -- skipping the port ${1} check" >&2
        return 0
    fi
    local holder
    holder="$(port_holder "$1")"
    if [[ -n "${holder}" ]]; then
        echo "[preflight] FAIL: port $1 is already in use, so $2 cannot start:" >&2
        echo "  ${holder}" >&2
        echo "  stop that process (or free the port) and re-run;" \
             "no pid shown means another user owns it: sudo ss -ltnp 'sport = :$1'" >&2
        return 1
    fi
}

# Wait up to $3 s for process $1 itself to listen on TCP port $2 (a foreign listener that took the
# port after require_port_free does not count); on failure print the tail of log $4.
wait_listening() {
    local pid="$1" port="$2" timeout_s="$3" log="$4" i
    command -v ss >/dev/null 2>&1 || return 0
    for ((i = 0; i < timeout_s * 10; i++)); do
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "[preflight] FAIL: process ${pid} exited before listening on :${port}; ${log}:" >&2
            tail -n 5 "${log}" >&2
            return 1
        fi
        port_holder "${port}" | grep -q "pid=${pid}," && return 0
        sleep 0.1
    done
    echo "[preflight] FAIL: process ${pid} is not listening on :${port} after ${timeout_s} s; ${log}:" >&2
    tail -n 5 "${log}" >&2
    return 1
}
