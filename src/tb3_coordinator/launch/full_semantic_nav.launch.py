"""
full_semantic_nav.launch.py — One-command launch for the full semantic navigation stack.

Composes:
  1. Gazebo simulation (warehouse_semantic world)
  2. SLAM Toolbox (online async)
  3. Nav2 navigation stack
  4. Frontier exploration (warmup + frontier detection + goal assignment)
  5. Semantic perception chain (detector + localizer + memory + query + nav_adapter)
  6. Coordinator (mode manager)

Usage:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch tb3_coordinator full_semantic_nav.launch.py

    # Then in another terminal:
    ros2 topic pub --once /user_command std_msgs/String "data: 'go to the person'"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Built-in Gazebo world presets shipped with tb3_frontier_exploration.
# Each entry maps a short alias (the value the user passes via
# `world:=...`) to (world_file, default_x, default_y).
#
# The default spawn pose is auto-applied when `x_pose` / `y_pose` are
# left at their sentinel "AUTO" values, so a bare
#   ros2 launch ... world:=warehouse_models_person
# already drops the bot at the recommended starting pose for that
# world. Pass `x_pose:=...`/`y_pose:=...` explicitly to override.
#
# Two officially supported worlds for the full semantic navigation stack:
#   - warehouse_models_person: **default**. 6×6 m room with five `person`
#                              figures (four corners + centre), sized so
#                              the LDS-01 LiDAR (3.5 m range) always sees
#                              every wall — fixes the SLAM "infinite
#                              corridor" problem and the duplicate
#                              landmark drift seen with larger rooms.
#                              Tuned for the "go to person N" workflow.
#   - warehouse_models:        the original 4×6 m room with one table +
#                              one person. Kept for backward
#                              compatibility and as a smaller test case.
#
# Other .world files (warehouse_semantic.world, detector_test.world)
# remain on disk for their own dedicated launches but are not exposed
# as aliases here. Pass an absolute path to use them via this launch.
WORLD_PRESETS = {
    "warehouse_models_person": {
        "file":      "warehouse_models_person.world",
        # (-1.5, 0): 1.5 m from the west wall (well clear of the default
        # Nav2 inflation_radius=0.55 m), 1.5 m from the centre person,
        # and 2.06 m from each of the NW/SW corner persons. Earlier
        # iterations used (-2.5, 0) which left only 0.5 m clearance to
        # the west wall — sometimes inside the inflation halo, which
        # made local_costmap mark the bot's spawn cell as "stuck in
        # obstacle" and caused planner_server to refuse to dispatch
        # the very first goal.
        "default_x": "-1.5",
        "default_y": "0.0",
    },
    "warehouse_models": {
        "file":      "warehouse_semantic_models.world",
        "default_x": "-1.2",
        "default_y": "-1.2",
    },
}

# Used as the fallback when the user passes a custom world (absolute
# path), since we have no way to know its valid spawn region.
_FALLBACK_SPAWN_X = "-1.2"
_FALLBACK_SPAWN_Y = "-1.2"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ── Package share dirs ────────────────────────────────────────────────
    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_det = get_package_share_directory("tb3_detector")
    pkg_loc = get_package_share_directory("tb3_localizer")
    pkg_mem = get_package_share_directory("tb3_memory")
    pkg_qry = get_package_share_directory("tb3_query")
    pkg_nav = get_package_share_directory("tb3_nav_adapter")
    pkg_coord = get_package_share_directory("tb3_coordinator")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # ── 1. Gazebo + TurtleBot3 ────────────────────────────────────────────
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    def make_gazebo_actions(context, *_args, **_kwargs):
        """Resolve `world:=...` and per-world spawn defaults, then build
        the Gazebo + TurtleBot3 launch actions accordingly.

        Runs at launch time (not at module import) so we can read the
        actual values of the `world` / `x_pose` / `y_pose`
        LaunchConfigurations via `.perform(context)`. We use
        OpaqueFunction here for two reasons:
          1. Dict lookup against WORLD_PRESETS needs the resolved string.
          2. We treat `x_pose=AUTO` / `y_pose=AUTO` as "use the per-world
             default" — distinguishing "user did not set it" from
             "user set it to a real number" requires the resolved
             string at launch time, which substitutions alone cannot
             give us.
        """
        raw = LaunchConfiguration("world").perform(context).strip()
        if not raw:
            raw = "warehouse_models_person"

        if raw in WORLD_PRESETS:
            preset = WORLD_PRESETS[raw]
            world_file = os.path.join(pkg_fe, "worlds", preset["file"])
            preset_x   = preset["default_x"]
            preset_y   = preset["default_y"]
            source = "alias"
        else:
            # Treat as a direct path. Accept absolute paths and paths
            # that exist relative to cwd. We deliberately do not silently
            # fall back to the default if the file is missing — fail
            # loudly so users notice typos.
            world_file = os.path.abspath(os.path.expanduser(raw))
            preset_x   = _FALLBACK_SPAWN_X
            preset_y   = _FALLBACK_SPAWN_Y
            source = "path"

        if not os.path.isfile(world_file):
            valid_aliases = ", ".join(sorted(WORLD_PRESETS.keys()))
            raise FileNotFoundError(
                f"[full_semantic_nav] world={raw!r} could not be resolved. "
                f"Tried as {source}: {world_file}. "
                f"Pass one of the built-in aliases ({valid_aliases}) or an "
                f"absolute path to a .world file."
            )

        # Resolve spawn pose. "AUTO" is the sentinel default declared
        # below; anything else is treated as a user-provided value.
        x_raw = LaunchConfiguration("x_pose").perform(context).strip()
        y_raw = LaunchConfiguration("y_pose").perform(context).strip()
        x_final = preset_x if x_raw == "AUTO" else x_raw
        y_final = preset_y if y_raw == "AUTO" else y_raw

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world_file}.items(),
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
            launch_arguments={"x_pose": x_final, "y_pose": y_final}.items(),
        )

        return [
            LogInfo(msg=(
                f"[full_semantic_nav] world={raw!r} → {world_file} ;"
                f" spawn=({x_final}, {y_final})"
            )),
            gzserver, gzclient, rsp, spawn,
        ]

    # ── 2. Nav2 + SLAM (integrated) ─────────────────────────────────────
    # Use Nav2 bringup with slam=True so it launches slam_toolbox internally
    # and does not require a pre-built map file.
    # Nav2 bringup: slam=True makes it launch slam_toolbox internally.
    # map arg is required even with slam=True; provide a dummy path that won't be loaded.
    # params_file must be explicit to avoid empty-path errors from ParameterFile.
    dummy_map = os.path.join(pkg_nav2, "maps", "turtlebot3_world.yaml")
    nav2_params = os.path.join(pkg_nav2, "params", "nav2_params.yaml")
    # Mute the very noisy `worldToMap failed: mx,my: ...` ERROR that
    # planner_server prints whenever the costmap inflation samples one cell
    # past the static-map boundary. It is benign in Nav2 Humble (planning
    # still succeeds and the goal is reached), but it floods the terminal
    # at planning rate.
    #
    # nav2_bringup forwards this single string verbatim as
    # `--ros-args --log-level <value>` to every Nav2 node. ROS 2's
    # `--log-level` supports a per-logger form `<logger>:=<level>`; when
    # passed a logger name that does not exist on the receiving node the
    # rcl logging machinery silently ignores it and the process default
    # stays at INFO. Therefore `planner_server:=fatal`:
    #   • on planner_server  → matches its node logger → silenced.
    #   • on every other Nav2 node (bt_navigator, controller_server, …)
    #     → no such logger → INFO defaults preserved, goal lifecycle and
    #     recoveries still print.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": dummy_map,
            "params_file": nav2_params,
            "autostart": "True",
            "use_composition": "False",
            "log_level": "planner_server:=fatal",
        }.items(),
    )

    # ── 4. Frontier exploration ───────────────────────────────────────────
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fe, "launch", "exploration.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 5. Semantic perception chain ──────────────────────────────────────
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_det, "launch", "detector.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, "launch", "localizer.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mem, "launch", "semantic_memory.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    query = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_qry, "launch", "semantic_query.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    nav_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "nav_goal_adapter.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 6. Coordinator ────────────────────────────────────────────────────
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_coord, "launch", "coordinator.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Compose with startup delays ──────────────────────────────────────
    # Gazebo + SLAM + Nav2 need time to initialize before exploration and
    # perception nodes start looking for topics and action servers.
    # ── 7. RViz (optional) ─────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare("tb3_coordinator"), "rviz", "semantic_nav.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # The .world files reference vendored Gazebo models (person_standing,
    # table_marble, stop_sign, ground_plane, sun) via model:// URIs. Prepend
    # the package-shipped models directory so a fresh machine never has to
    # hit the (deprecated, slow) online Gazebo model database.
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            os.path.join(pkg_fe, "models"),
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with semantic nav config"),

        # Must come before the Gazebo actions below.
        set_gazebo_model_path,

        # Spawn pose defaults to "AUTO" — at launch time the OpaqueFunction
        # below substitutes in the per-world default registered in
        # WORLD_PRESETS (or `_FALLBACK_SPAWN_*` for custom .world paths).
        # Pass `x_pose:=<float>` / `y_pose:=<float>` to override.
        DeclareLaunchArgument(
            "x_pose", default_value="AUTO",
            description=(
                "Robot spawn x in map frame, or 'AUTO' to use the "
                "world preset's recommended pose."
            ),
        ),
        DeclareLaunchArgument(
            "y_pose", default_value="AUTO",
            description=(
                "Robot spawn y in map frame, or 'AUTO' to use the "
                "world preset's recommended pose."
            ),
        ),

        # Pick the Gazebo world. Accepts a short alias
        # (warehouse_models_person, warehouse_models) or an absolute
        # path to a .world file. Default is the 5-person warehouse used
        # by the "go to person N" demo. See WORLD_PRESETS at the top of
        # this file for the alias table.
        DeclareLaunchArgument(
            "world",
            default_value="warehouse_models_person",
            description=(
                "Gazebo world: alias (warehouse_models_person | "
                "warehouse_models) or absolute path to a .world file."
            ),
        ),

        # Phase 1: simulation + navigation infrastructure
        # Nav2 (autostart=True) needs Gazebo + SLAM to publish /map before
        # planner_server can finish configure(); 10s is a safer buffer than 5s
        # to absorb CPU jitter (otherwise lifecycle_manager hangs on
        # "Waiting for service planner_server/get_state...").
        OpaqueFunction(function=make_gazebo_actions),
        TimerAction(period=10.0, actions=[nav2]),

        # Phase 2: exploration + perception (after Nav2 has time to start)
        TimerAction(period=20.0, actions=[exploration]),
        TimerAction(period=15.0, actions=[detector]),
        TimerAction(period=15.0, actions=[localizer]),
        TimerAction(period=15.0, actions=[memory]),
        TimerAction(period=15.0, actions=[query]),
        TimerAction(period=15.0, actions=[nav_adapter]),

        # Phase 3: coordinator (after everything else is up)
        TimerAction(period=23.0, actions=[coordinator]),

        # Persistent semantic map memory + RViz visualization
        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_map_memory_node",
                name="semantic_map_memory_node",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("tb3_coordinator"), "config", "coordinator.yaml"
                    ]),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]),

        # RViz (immediate, gated by use_rviz arg)
        rviz_node,

        # Runtime debug diagnostics (gated by use_runtime_debug arg)
        DeclareLaunchArgument("use_runtime_debug", default_value="false",
                              description="Launch semantic runtime debug node"),
        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_runtime_debug_node",
                name="semantic_runtime_debug_node",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_runtime_debug")),
                output="screen",
            ),
        ]),
    ])
