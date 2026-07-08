"""Launch the micropilot_rendering_node LifecycleNode.

Loads config/default_params.yaml by default. Switch deployments by config NAME
(a file inside this package's config/ directory):
  ros2 launch micropilot_rendering_node rendering_node.launch.py config:=m2o1_params.yaml
Or override with a full path:
  ros2 launch micropilot_rendering_node rendering_node.launch.py \\
      params_file:=/path/to/your_robot_params.yaml

The node starts in 'unconfigured' state. The smoke test (test/smoke_test.py)
triggers configure + activate programmatically. For manual activation:
  ros2 lifecycle set /rendering_node configure
  ros2 lifecycle set /rendering_node activate
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("micropilot_rendering_node"), "config", LaunchConfiguration("config")]
    )

    # Subscribe to the 6 CARLA camera streams over Iceoryx shared memory
    # (zero-copy) instead of loopback UDP. SHM only engages when BOTH this node
    # AND the publisher (CARLA bridge) have it enabled; without it, every 4 MB
    # frame is copied per-subscriber and collapses the synchronous-mode sim FPS.
    # Set here so it holds regardless of the launching shell. Skip if already set.
    pre_actions = []
    if "CYCLONEDDS_URI" not in os.environ:
        pre_actions.append(SetEnvironmentVariable(
            "CYCLONEDDS_URI",
            "file://" + os.path.expanduser("~/.config/cyclonedds/cyclonedds.xml")))

    args = [
        DeclareLaunchArgument(
            "config",
            default_value="default_params.yaml",
            description="Config file NAME inside this package's config/ dir "
                        "(default_params.yaml = CARLA sim, m2o1_params.yaml = real robot).",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Node parameters YAML (full path; overrides config:=).",
        ),
    ]

    node = LifecycleNode(
        package="micropilot_rendering_node",
        executable="rendering_node",
        name="rendering_node",
        namespace="",
        # config_path: the resolved YAML fed to this node — read by the WS
        # bridge's save_params so the GUI tuning panel can update it in place.
        parameters=[LaunchConfiguration("params_file"),
                    {"config_path": LaunchConfiguration("params_file")}],
        output="screen",
    )

    return LaunchDescription(pre_actions + args + [node])
