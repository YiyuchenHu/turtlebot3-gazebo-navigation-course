# `warehouse_semantic.world`

Minimal Gazebo Classic warehouse (≈8 m × 10 m interior) for TurtleBot3 + SLAM + Nav2 experiments.
Note: the main course worlds are `warehouse_models_person.world` (default) and
`warehouse_semantic_models.world` — this empty room is kept for SLAM/Nav2-only testing.

## Coordinate layout

- **World frame:** Gazebo world origin at the **center** of the floor opening.
- **Axes:** +X east, +Y north, +Z up (Gazebo default).
- **Interior:** roughly **x ∈ [-4, 4]**, **y ∈ [-5, 5]** (8 m × 10 m clear floor inside the walls).

## Recommended robot spawn

- **Pose:** `(0, 0, 0)` yaw `0` — center of the warehouse, facing +X.
- Matches typical TurtleBot3 empty-world spawn and keeps semantic coordinates easy to reason about.

## Launch with TurtleBot3 (ROS 2 Humble)

**Recommended — package launch** (uses `warehouse_semantic.world` + gzserver/gzclient + spawn):

```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration warehouse_semantic_sim.launch.py use_sim_time:=true x_pose:=0.0 y_pose:=0.0
```

Note: upstream `turtlebot3_gazebo/empty_world.launch.py` **hardcodes** its world path on Humble; it does not accept `world:=...`. Use the launch above or `gazebo_ros` manually:

```bash
WORLD=$(ros2 pkg prefix tb3_frontier_exploration)/share/tb3_frontier_exploration/worlds/warehouse_semantic.world
ros2 launch gazebo_ros gzserver.launch.py world:=$WORLD
# other terminal:
ros2 launch gazebo_ros gzclient.launch.py
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0
```

## Notes

- Walls are **static**; thickness **0.2 m**, height **2.5 m** — visible to lidar and safe for collision.
- **Optional later:** add boxes or shelves as separate `<model>` blocks; keep them commented or in a second world file to avoid cluttering v1.
