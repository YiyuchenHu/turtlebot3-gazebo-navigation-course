import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_frontier_exploration")
    config = os.path.join(pkg_share, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_topic = LaunchConfiguration("map_topic", default="/map")
    costmap_topic = LaunchConfiguration("costmap_topic", default="/global_costmap/costmap")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                             description="Use simulation time"),
        DeclareLaunchArgument("map_topic", default_value="/map",
                             description="OccupancyGrid topic for the map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap",
                             description="Costmap topic for frontier cost sampling"),

        Node(
            package="tb3_frontier_exploration",
            executable="frontier_detection_node",
            name="frontier_detection_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                {"frontier_detection_node": {"ros__parameters": {
                    "map_topic": map_topic,
                    "costmap_topic": costmap_topic,
                }}},
            ],
            output="screen",
        ),
    ])
