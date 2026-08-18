"""
Shim for the stock TurtleBot3 Gazebo world.

tb3_frontier_exploration did not ship this file before; use this when you want
``ros2 launch tb3_frontier_exploration turtlebot3_world.launch.py`` instead of
invoking turtlebot3_gazebo by package name.

Actual sim stack lives in turtlebot3_gazebo (turtlebot3_world.world).
Requires: gazebo_ros, turtlebot3_gazebo. Set TURTLEBOT3_MODEL (e.g. waffle_pi).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Publish clock from /clock (simulation).",
        ),
        DeclareLaunchArgument(
            "x_pose",
            default_value="-2.0",
            description="Spawn x in Gazebo world frame (m).",
        ),
        DeclareLaunchArgument(
            "y_pose",
            default_value="-0.5",
            description="Spawn y in Gazebo world frame (m).",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_tb3, "turtlebot3_world.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "x_pose": x_pose,
                "y_pose": y_pose,
            }.items(),
        ),
    ])
