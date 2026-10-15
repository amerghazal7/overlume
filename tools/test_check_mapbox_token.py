# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Offline tests for tools/check_mapbox_token.sh: a stub curl stands in for the network.

Run: python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
"""

import os
import subprocess

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_mapbox_token.sh")
TOKEN = "pk.test-token-not-real"
LANDMARK_URL = ("https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/"
                "14/10707/7006.glb")
FOOTPRINT_URL = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/42830/28025.vector.pbf"

# Logs its argv and the config it reads on stdin, then answers like `curl -w '%{http_code}'`.
# STUB_LANDMARK and STUB_FOOTPRINT hold "<http code>:<curl exit status>".
STUB_CURL = r"""#!/bin/sh
config=$(cat)
printf 'argv: %s\n%s\n' "$*" "$config" >> "$STUB_LOG"
case "$config" in
    *3dtiles*) reply=$STUB_LANDMARK ;;
    *) reply=$STUB_FOOTPRINT ;;
esac
printf '%s' "${reply%%:*}"
exit "${reply#*:}"
"""


def run_check(tmp_path, token, landmark="200:0", footprint="200:0"):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "curl").write_text(STUB_CURL)
    (bin_dir / "curl").chmod(0o755)
    env = {k: v for k, v in os.environ.items() if k != "MAPBOX_TOKEN"}
    env.update(PATH=f"{bin_dir}{os.pathsep}{env['PATH']}", STUB_LOG=str(tmp_path / "curl.log"),
               STUB_LANDMARK=landmark, STUB_FOOTPRINT=footprint)
    if token is not None:
        env["MAPBOX_TOKEN"] = token
    return subprocess.run(["bash", SCRIPT], env=env, stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=30)


@pytest.mark.parametrize("token", [None, "", " \n"])
def test_missing_token_skips_without_a_request(tmp_path, token):
    r = run_check(tmp_path, token)
    assert r.returncode == 2
    assert r.stdout == "SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)\n"
    assert r.stderr == ""
    assert not (tmp_path / "curl.log").exists()


def test_both_tiles_200_pass(tmp_path):
    r = run_check(tmp_path, TOKEN)
    assert r.returncode == 0, r.stdout
    assert r.stdout == ("PASS: token retrieves the Downtown Dubai tiles "
                        "(landmark tile HTTP 200, footprint tile HTTP 200)\n")
    assert TOKEN not in r.stdout + r.stderr


@pytest.mark.parametrize("landmark,footprint,codes", [
    ("401:0", "401:0", "landmark tile HTTP 401, footprint tile HTTP 401"),
    ("204:0", "200:0", "landmark tile HTTP 204, footprint tile HTTP 200"),
    ("200:0", "000:28", "landmark tile HTTP 200, footprint tile transport error (curl exit 28)"),
], ids=["both-401", "landmark-204", "footprint-timeout"])
def test_anything_but_two_200s_fails(tmp_path, landmark, footprint, codes):
    r = run_check(tmp_path, TOKEN, landmark, footprint)
    assert r.returncode == 1
    assert r.stdout == f"FAIL: {codes} (see docs/runbooks/mapbox.md)\n"


def test_token_reaches_curl_only_on_stdin_and_trimmed(tmp_path):
    # The library trims surrounding whitespace, so the preflight sends what the node will send.
    r = run_check(tmp_path, f" {TOKEN}\n", landmark="401:0", footprint="000:6")
    assert TOKEN not in r.stdout + r.stderr
    log = (tmp_path / "curl.log").read_text().splitlines()
    assert [line for line in log if line.startswith("argv: ")] == [
        "argv: -s --max-time 30 --config - -o /dev/null -w %{http_code}"] * 2
    assert [line for line in log if line.startswith("url = ")] == [
        f'url = "{LANDMARK_URL}?access_token={TOKEN}"',
        f'url = "{FOOTPRINT_URL}?access_token={TOKEN}"',
    ]
