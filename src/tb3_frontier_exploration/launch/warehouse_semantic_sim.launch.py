"""
Gazebo Classic + TurtleBot3 in warehouse_semantic.world.
Requires: gazebo_ros, turtlebot3_gazebo. Set TURTLEBOT3_MODEL before launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    pkg_tb3_fe = get_package_share_directory("tb3_frontier_exploration")
    world = os.path.join(pkg_tb3_fe, "worlds", "warehouse_semantic.world")

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    # Defaults inside 4 m × 6 m floor (inner x ∈ [-2, 2], y ∈ [-3, 3]).
    x_pose = LaunchConfiguration("x_pose", default="-1.2")
    y_pose = LaunchConfiguration("y_pose", default="-1.2")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="-1.2"),
        DeclareLaunchArgument("y_pose", default_value="-1.2"),
        # Vendored Gazebo models (model:// URIs in the .world files).
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[
                os.path.join(pkg_tb3_fe, "models"),
                ":",
                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            ],
        ),
        gzserver,
        gzclient,
        rsp,
        spawn,
    ])
