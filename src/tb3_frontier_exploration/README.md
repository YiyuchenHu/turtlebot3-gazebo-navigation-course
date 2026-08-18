# tb3_frontier_exploration

TurtleBot3 custom frontier-based autonomous exploration package (Phase 2).

## Nodes

- **frontier_detection_node** — Detects frontiers from `/map` or costmap; publishes frontier list.
- **goal_assignment_node** — Selects next frontier and sends goals via Nav2 `navigate_to_pose`.

## Launch

- `frontier_detection.launch.py` — Run frontier detection only.
- `goal_assignment.launch.py` — Run goal assignment only.
- `exploration.launch.py` — Run both (full exploration pipeline).

## Dependencies

ROS2 Humble, Nav2, slam_toolbox (or map source). See `package.xml` for message/action dependencies.

## Config

`config/params.yaml` — Parameters for both nodes (topics, frame_id, timeouts).
