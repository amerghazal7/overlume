"""Unit test for the WS bridge command parsing (no ROS needed).

Run: pytest tools/test_vcam_ws_bridge.py
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from vcam_ws_bridge import parse_cmd, patch_yaml_text  # noqa: E402


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
    '{"cmd": "set_param", "name": "nope", "value": 1}',   # untunable param
    '{"cmd": "set_param", "name": "bowl_R0", "value": "6"}',
    '{"cmd": "set_param", "name": "splat_radius", "value": 2.5}',
    '{"cmd": "set_param", "name": "fill_blind_zone", "value": 1}',
    '{"cmd": "set_param", "name": "camera_extrinsics", "value": []}',
    '{"cmd": "save_params", "path": ""}',
])
def test_rejects_malformed(text):
    with pytest.raises(ValueError):
        parse_cmd(text)


def test_param_cmds_valid():
    assert parse_cmd('{"cmd": "get_params"}') == ("get_params", None)
    assert parse_cmd('{"cmd": "set_param", "name": "bowl_R0", "value": 6}') == \
        ("set_param", ("bowl_R0", 6.0))
    assert parse_cmd('{"cmd": "set_param", "name": "splat_radius", "value": 3}') == \
        ("set_param", ("splat_radius", 3))
    assert parse_cmd(
        '{"cmd": "set_param", "name": "exposure_match", "value": false}') == \
        ("set_param", ("exposure_match", False))
    cmd, (name, v) = parse_cmd(
        '{"cmd": "set_param", "name": "camera_extrinsics", "value": [1, 0, 2.5]}')
    assert v == [1.0, 0.0, 2.5]
    assert parse_cmd('{"cmd": "save_params"}') == ("save_params", None)
    assert parse_cmd('{"cmd": "save_params", "path": "/tmp/x.yaml"}') == \
        ("save_params", "/tmp/x.yaml")


YAML = """# header comment
/**:
  ros__parameters:
    n_cameras: 6
    bowl_R0: 6.0  # tuned
    fill_blind_zone: true
    camera_extrinsics:
    - 1.0
    - 2.0
    image_topics:
    - /a
    - /b
"""


def test_patch_yaml_scalar_and_list():
    out = patch_yaml_text(YAML, {"bowl_R0": 9.5, "fill_blind_zone": False,
                                 "camera_extrinsics": [3.0, 4.0, 5.0]})
    assert "# header comment" in out
    assert "    bowl_R0: 9.5\n" in out
    assert "    fill_blind_zone: false\n" in out
    assert "    - 3\n" in out and "    - 5\n" in out and "- 1.0" not in out
    assert "- /a" in out and "- /b" in out          # untouched string list
    assert "    n_cameras: 6" in out


def test_patch_yaml_appends_missing_key():
    out = patch_yaml_text(YAML, {"exposure_match": True})
    assert "    exposure_match: true\n" in out


def test_pose_roundtrip():
    pytest.importorskip("gi")
    np = pytest.importorskip("numpy")
    from vcam_gui import pose_to_rt, rt_to_pose
    # a real bl-style row: yawed back-left, pitched slightly down
    row = pose_to_rt(0.21, 0.38, 0.72, 139.0, -2.2, 1.5)
    back = rt_to_pose(row)
    assert np.allclose(back, [0.21, 0.38, 0.72, 139.0, -2.2, 1.5], atol=1e-6)
    R = np.array(row[:9]).reshape(3, 3)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)  # orthonormal
