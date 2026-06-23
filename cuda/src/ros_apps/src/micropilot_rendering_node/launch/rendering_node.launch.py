"""Launch the micropilot_rendering_node LifecycleNode.

Usage:
  ros2 launch micropilot_rendering_node rendering_node.launch.py \\
      n_cameras:=4 out_width:=640 out_height:=480

The node starts in 'unconfigured' state.  The smoke test (test/smoke_test.py)
triggers configure + activate programmatically.  For manual activation:
  ros2 lifecycle set /rendering_node configure
  ros2 lifecycle set /rendering_node activate
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    # ── launch arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("n_cameras", default_value="4",
                              description="Number of real cameras"),
        DeclareLaunchArgument("out_width",  default_value="640",
                              description="Output virtual-camera image width"),
        DeclareLaunchArgument("out_height", default_value="480",
                              description="Output virtual-camera image height"),
        DeclareLaunchArgument("bowl_R0",   default_value="6.0",
                              description="Bowl flat-floor radius (m)"),
        DeclareLaunchArgument("bowl_k",    default_value="0.08",
                              description="Bowl wall curvature"),
        DeclareLaunchArgument("bowl_Rmax", default_value="20.0",
                              description="Bowl max radius (m)"),
    ]

    # ── node ─────────────────────────────────────────────────────────────────
    node = LifecycleNode(
        package="micropilot_rendering_node",
        executable="rendering_node",
        name="rendering_node",
        namespace="",
        parameters=[
            {
                "n_cameras":  LaunchConfiguration("n_cameras"),
                "out_width":  LaunchConfiguration("out_width"),
                "out_height": LaunchConfiguration("out_height"),
                "bowl_R0":    LaunchConfiguration("bowl_R0"),
                "bowl_k":     LaunchConfiguration("bowl_k"),
                "bowl_Rmax":  LaunchConfiguration("bowl_Rmax"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(args + [node])
