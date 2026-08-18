"""
semantic_memory.launch.py — Launch the Stage-3 semantic memory node.

Usage:
    ros2 launch tb3_memory semantic_memory.launch.py
    ros2 launch tb3_memory semantic_memory.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_memory")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running with Gazebo simulation.",
        ),

        Node(
            package="tb3_memory",
            executable="semantic_memory_node",
            name="semantic_memory_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "semantic_memory.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
