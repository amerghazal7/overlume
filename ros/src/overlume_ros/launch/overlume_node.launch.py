"""Launch the overlume_ros LifecycleNode.

Loads config/default_params.yaml by default:
  ros2 launch overlume_ros overlume_node.launch.py
Or override with a full path:
  ros2 launch overlume_ros overlume_node.launch.py \\
      params_file:=/path/to/your_params.yaml

The node starts in 'unconfigured' state. The smoke test (test/smoke_test.py)
triggers configure + activate programmatically. For manual activation:
  ros2 lifecycle set /overlume_node configure
  ros2 lifecycle set /overlume_node activate

Mirrors micropilot_rendering_node/launch/rendering_node.launch.py's
autostart-lifecycle pattern so both mode-mux nodes come up the same way.

Task 4 (VM-093), Global Constraints (`rendering_node.launch.py:36-45`, fix
commit `69b5a3a`): this node now ingests the same 6 raw CARLA camera streams
(camera_ingest.cpp, VM-091) rendering_node always has, over the identical
CYCLONEDDS_URI shared-memory transport -- a transport-layer fact, unrelated
to which process subscribes, that is trivial to silently lose when copying
launch files (rendering_node.launch.py's own comment: default loopback-UDP
collapsed sim FPS 32->8 Hz for 6x ~4 MB frames). Carried forward here
identically, not reinvented.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("overlume_ros"), "config", "default_params.yaml"]
    )

    # Same SHM-forcing config as rendering_node.launch.py:36-45, carried
    # forward verbatim (Global Constraints / Task 4 Step 2) -- SHM only
    # engages when BOTH this node AND the publisher (CARLA bridge) have it
    # enabled; without it, every ~4 MB frame is copied per-subscriber and
    # collapses the synchronous-mode sim FPS. Skip if already set.
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
        parameters=[LaunchConfiguration("params_file")],
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

    return LaunchDescription(pre_actions + args + [node] + auto)
