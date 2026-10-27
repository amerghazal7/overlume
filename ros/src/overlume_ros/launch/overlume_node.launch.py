# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Launch the overlume_ros LifecycleNode.

Loads config/default_params.yaml by default:
  ros2 launch overlume_ros overlume_node.launch.py
Layer a params profile over it (config/<name>_params.yaml, or a path, overrides the keys it
names and leaves the rest):
  ros2 launch overlume_ros overlume_node.launch.py params_profile:=/path/to/robot_params.yaml
Or replace the defaults with a full path:
  ros2 launch overlume_ros overlume_node.launch.py \\
      params_file:=/path/to/your_params.yaml

The node starts in 'unconfigured' state. The smoke test (test/smoke_test.py)
triggers configure + activate programmatically. For manual activation:
  ros2 lifecycle set /overlume_node configure
  ros2 lifecycle set /overlume_node activate

Autostart-lifecycle pattern.

Task 4 (VM-093), Global Constraints (`rendering_node.launch.py:36-45`, fix
commit `42aa148`): this node now ingests the same 6 raw CARLA camera streams
(camera_ingest.cpp, VM-091) rendering_node always has, over the identical
CYCLONEDDS_URI shared-memory transport -- a transport-layer fact, unrelated
to which process subscribes, that is trivial to silently lose when copying
launch files (rendering_node.launch.py's own comment: default loopback-UDP
collapsed sim FPS 32->8 Hz for 6x ~4 MB frames). Carried forward here
identically, not reinvented.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
                            SetEnvironmentVariable, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition

def profile_params_file(params_file: str, profile: str, config_dir: str) -> str:
    """The file the node loads after params_file: config/<profile>_params.yaml for a name, the path
    itself for a path, and params_file again (a no-op) without a profile, so that a user's own
    params_file is never overridden by anything they did not ask for."""
    if not profile:
        return params_file
    if profile.endswith(".yaml") or "/" in profile:
        return profile
    return os.path.join(config_dir, f"{profile}_params.yaml")


def _resolve_params_profile(context):
    return [SetLaunchConfiguration("params_profile_file", profile_params_file(
        LaunchConfiguration("params_file").perform(context),
        LaunchConfiguration("params_profile").perform(context),
        os.path.join(get_package_share_directory("overlume_ros"), "config")))]


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("overlume_ros"), "config", "default_params.yaml"]
    )

    pre_actions = []
    if "CYCLONEDDS_URI" not in os.environ:
        pre_actions.append(SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            "file://" + os.path.expanduser("~/.config/cyclonedds/cyclonedds.xml")))

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Node parameters YAML (full path).",
        ),
        DeclareLaunchArgument(
            "params_profile",
            default_value="",
            description="config/<name>_params.yaml (or a path), loaded after params_file so "
                        "its keys win. Empty: params_file alone.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure + activate the lifecycle node "
                        "on startup. Set false for manual lifecycle control.",
        ),
    ]

    node = LifecycleNode(
        package="overlume_ros",
        executable="overlume_node",
        name="overlume_node",
        namespace="",
        parameters=[LaunchConfiguration("params_file"),
                    LaunchConfiguration("params_profile_file")],
        output="screen",
    )

    auto = [
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE),
            condition=IfCondition(LaunchConfiguration("autostart")),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE))],
            ),
            condition=IfCondition(LaunchConfiguration("autostart")),
        ),
    ]

    return LaunchDescription(
        pre_actions + args + [OpaqueFunction(function=_resolve_params_profile), node] + auto)
