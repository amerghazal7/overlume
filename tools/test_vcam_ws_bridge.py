"""Unit test for the WS bridge command parsing (no ROS needed).

Run: pytest tools/test_vcam_ws_bridge.py
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from vcam_ws_bridge import parse_cmd  # noqa: E402


def test_set_look_valid():
    cmd, (eye, target) = parse_cmd(json.dumps(
        {"cmd": "set_look", "eye": [1, 2, 3.5], "target": [0, 0, 0.3]}))
    assert cmd == "set_look"
    assert eye == [1.0, 2.0, 3.5]
    assert target == [0.0, 0.0, 0.3]


def test_set_preset_valid():
    assert parse_cmd('{"cmd": "set_preset", "preset": 5}') == ("set_preset", 5)


@pytest.mark.parametrize("mode,expect", [
    ("bowl", 1), ("pointcloud", 2), (1, 1), (2, 2),
])
def test_set_render_mode_valid(mode, expect):
    assert parse_cmd(json.dumps({"cmd": "set_render_mode", "mode": mode})) == \
        ("set_render_mode", expect)


@pytest.mark.parametrize("text", [
    "not json",
    "[1,2,3]",                                            # not an object
    '{"cmd": "warp"}',                                    # unknown cmd
    '{"cmd": "set_look", "eye": [1, 2], "target": [0, 0, 0]}',       # short eye
    '{"cmd": "set_look", "eye": [1, 2, "x"], "target": [0, 0, 0]}',  # non-number
    '{"cmd": "set_look", "eye": [1, 2, 3]}',              # missing target
    '{"cmd": "set_preset", "preset": 0}',                 # out of range
    '{"cmd": "set_preset", "preset": 6}',
    '{"cmd": "set_preset", "preset": true}',              # bool is not an index
    '{"cmd": "set_preset", "preset": "2"}',
    '{"cmd": "set_render_mode", "mode": 3}',              # unknown mode
    '{"cmd": "set_render_mode", "mode": "depth"}',
    '{"cmd": "set_render_mode", "mode": true}',           # bool is not a mode
])
def test_rejects_malformed(text):
    with pytest.raises(ValueError):
        parse_cmd(text)
