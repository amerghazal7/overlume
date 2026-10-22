#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""The launch file's params_profile resolution.

Usage: test_launch_params.py <overlume_node.launch.py>. Without a profile the node loads
params_file alone (a user's own file must never be overridden by default_params.yaml); a name
loads config/<name>_params.yaml after it, and a path loads that path after it.
"""
import importlib.util
import sys


def main(launch_path):
    spec = importlib.util.spec_from_file_location("overlume_node_launch", launch_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    f = mod.profile_params_file
    cfg = "/share/overlume_ros/config"
    assert f("/my/robot.yaml", "", cfg) == "/my/robot.yaml", "no profile: params_file alone"
    assert f("/d.yaml", "robot", cfg) == "/share/overlume_ros/config/robot_params.yaml"
    assert f("/d.yaml", "/p/mine.yaml", cfg) == "/p/mine.yaml"
    assert f("/d.yaml", "mine.yaml", cfg) == "mine.yaml"
    print("PASS: params_profile resolves as documented")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
