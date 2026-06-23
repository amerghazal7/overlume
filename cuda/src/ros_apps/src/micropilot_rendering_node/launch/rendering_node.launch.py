"""Launch the micropilot_rendering_node LifecycleNode.

Loads config/default_params.yaml by default. Override per deployment:
  ros2 launch micropilot_rendering_node rendering_node.launch.py \\
      params_file:=/path/to/your_robot_params.yaml

The node starts in 'unconfigured' state. The smoke test (test/smoke_test.py)
triggers configure + activate programmatically. For manual activation:
  ros2 lifecycle set /rendering_node configure
  ros2 lifecycle set /rendering_node activate
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("micropilot_rendering_node"), "config", "default_params.yaml"]
    )

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Node parameters YAML (see config/default_params.yaml).",
        ),
    ]

    node = LifecycleNode(
        package="micropilot_rendering_node",
        executable="rendering_node",
        name="rendering_node",
        namespace="",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription(args + [node])
