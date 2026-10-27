# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Checks for the ego-model provisioning scripts (no ROS needed).

Run: python3 -m pytest tools/test_ego_provisioning.py -q -p no:anyio
"""

import pathlib
import shutil
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
PROVISION = REPO / "ros/src/overlume_ros/scripts/provision_ego_model.sh"
OBJ2GLTF = REPO / "overlume/scripts/obj2gltf.py"


def test_provision_without_argument_prints_usage():
    r = subprocess.run(["bash", str(PROVISION)], capture_output=True, text=True)
    assert r.returncode == 2
    assert "usage: provision_ego_model.sh" in r.stderr


def test_provision_copies_model_to_ego_glb(tmp_path):
    (tmp_path / "scripts").mkdir()
    script = tmp_path / "scripts" / PROVISION.name
    shutil.copy(PROVISION, script)
    model = tmp_path / "any_name.glb"
    model.write_bytes(b"glTF-dummy")
    r = subprocess.run(["bash", str(script), str(model)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "assets/ego/ego.glb").read_bytes() == b"glTF-dummy"


def test_obj2gltf_without_arguments_prints_usage():
    pytest.importorskip("trimesh")  # the script imports trimesh before it checks argv
    r = subprocess.run([sys.executable, str(OBJ2GLTF)], capture_output=True, text=True)
    assert r.returncode != 0
    assert "obj2gltf" in r.stdout + r.stderr
