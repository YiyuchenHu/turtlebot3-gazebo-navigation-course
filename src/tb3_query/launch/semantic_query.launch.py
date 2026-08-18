"""
semantic_query.launch.py — Launch the Stage-4 semantic query node.

Usage:
    ros2 launch tb3_query semantic_query.launch.py
    ros2 launch tb3_query semantic_query.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_query")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running with Gazebo simulation.",
        ),

        Node(
            package="tb3_query",
            executable="semantic_query_node.py",
            name="semantic_query_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "semantic_query.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
