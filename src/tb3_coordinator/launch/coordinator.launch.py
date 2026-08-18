"""
coordinator.launch.py — Launch the semantic navigation coordinator.

Usage:
    ros2 launch tb3_coordinator coordinator.launch.py
    ros2 launch tb3_coordinator coordinator.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_coordinator")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running with Gazebo simulation.",
        ),

        Node(
            package="tb3_coordinator",
            executable="coordinator_node",
            name="coordinator_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "coordinator.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
