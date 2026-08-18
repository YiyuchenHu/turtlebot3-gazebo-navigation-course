"""
nav_goal_adapter.launch.py — Launch the Stage-5 nav goal adapter node.

Usage:
    ros2 launch tb3_nav_adapter nav_goal_adapter.launch.py
    ros2 launch tb3_nav_adapter nav_goal_adapter.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_nav_adapter")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running with Gazebo simulation.",
        ),

        Node(
            package="tb3_nav_adapter",
            executable="nav_goal_adapter_node",
            name="nav_goal_adapter_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "nav_goal_adapter.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
