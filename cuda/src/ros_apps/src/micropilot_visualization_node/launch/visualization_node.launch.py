"""Launch the micropilot_visualization_node LifecycleNode.

Loads config/default_params.yaml by default:
  ros2 launch micropilot_visualization_node visualization_node.launch.py
Or override with a full path:
  ros2 launch micropilot_visualization_node visualization_node.launch.py \\
      params_file:=/path/to/your_params.yaml

The node starts in 'unconfigured' state. The smoke test (test/smoke_test.py)
triggers configure + activate programmatically. For manual activation:
  ros2 lifecycle set /visualization_node configure
  ros2 lifecycle set /visualization_node activate

Mirrors micropilot_rendering_node/launch/rendering_node.launch.py's
autostart-lifecycle pattern so both mode-mux nodes come up the same way.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
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
        [FindPackageShare("micropilot_visualization_node"), "config", "default_params.yaml"]
    )

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
        package="micropilot_visualization_node",
        executable="visualization_node",
        name="visualization_node",
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

    return LaunchDescription(args + [node] + auto)
