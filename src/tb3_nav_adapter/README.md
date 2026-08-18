# `tb3_nav_adapter`

`tb3_nav_adapter` is Stage 5 of the TurtleBot3 semantic navigation pipeline.

Current pipeline:

```text
camera -> detector -> localizer -> semantic memory -> semantic query -> nav goal adapter
```

Stage 4 answers:

> Which remembered object does the user mean?

Stage 5 answers:

> How do we turn that selected semantic object into a goal pose for navigation?

`tb3_nav_adapter` converts the output of semantic query into a safe approach `PoseStamped` goal that can be consumed by Nav2 or inspected manually. It has already been implemented as an MVP and validated in practice.

## What `tb3_nav_adapter` Does

`tb3_nav_adapter` receives a selected semantic target from Stage 4 and generates a navigation-ready goal pose.

In practical terms, it:

1. receives a `SemanticQueryResult` from `/semantic_query_node/selected_target`
2. validates that the query was successful
3. computes a safe approach position offset back from the target
4. orients the goal so the robot will face the target on arrival
5. attempts to transform the goal from `base_link` to `map` via TF
6. falls back to `base_link` if `map` is not available
7. publishes the resulting `PoseStamped` on `/nav_goal_adapter_node/goal_pose`

## What It Does Not Do Yet

The current nav goal adapter MVP does **not** do the following:

- no direct `NavigateToPose` action execution yet
- no full Nav2 orchestration yet
- no behavior tree logic
- no advanced obstacle-aware goal optimization yet
- no object-specific grasping or interaction logic
- no full map-frame world-model reasoning beyond simple TF conversion with fallback

The current scope is goal generation only. Execution is left to downstream Nav2 integration.

## Why Nav Goal Adaptation Is Needed

Semantic query output alone is not enough for navigation.

Selecting an object and navigating to it are different problems:

- the selected object position is **where the object is**, not where the robot should go
- driving directly to the object point would collide with it
- navigation needs a safe approach pose with a stand-off distance
- navigation needs an orientation so the robot faces the target on arrival
- table-like objects may localize to a leg or LiDAR return rather than a geometric center, so the generated goal should be treated as an approach point near the object, not a precision endpoint

## Input Topic

### `/semantic_query_node/selected_target`

- Type: `tb3_query/msg/SemanticQueryResult`
- Source: Stage 4 semantic query

The adapter uses these fields from the query result:

- `success` — whether the query found a valid target (skip if false)
- `semantic_name` — for logging
- `detector_label` — for logging
- `object_id` — for logging and tracking
- `position` — the `(x, y)` of the target in the source frame
- `frame_id` — the coordinate frame of the position (typically `base_link`)
- `confidence` — not directly used in goal math, but available for future filtering

## Output Topic

### `/nav_goal_adapter_node/goal_pose`

- Type: `geometry_msgs/msg/PoseStamped`
- Published once per successful query result

This is the MVP output for downstream navigation. It can later be connected directly to a Nav2 `NavigateToPose` action client. The current implementation focuses on goal generation, not action execution.

## Goal Generation Principle

Given a target at `(tx, ty)` in base_link:

### Step 1: Compute direction from robot to target

```text
direction = atan2(ty, tx)
```

### Step 2: Step back by approach distance

```text
goal_x = tx - approach_distance * cos(direction)
goal_y = ty - approach_distance * sin(direction)
```

### Step 3: Orient the robot to face the target

```text
goal_yaw = direction
```

In words:

- the adapter first computes the direction from the robot origin to the target
- then it steps back along that direction by a configurable `approach_distance`
- the resulting position is the navigation goal
- the orientation is set so the robot will face the target on arrival

### Why this offset is important

If the robot navigated directly to the target point, it would drive into the object. The approach offset ensures the robot stops a safe distance away.

This is especially important for table/bench-like targets, where the localized point may correspond to a table leg or the nearest LiDAR return, not the geometric center of the table. The approach pose gets the robot near the object and facing it, which is good enough for the current MVP.

### Safety clamp

If the target is closer than `min_standoff_distance`, the goal is rejected entirely to avoid collision. If the target is between `min_standoff_distance` and `approach_distance + min_standoff_distance`, the approach offset is clamped so the goal stays at least `min_standoff_distance` from the target.

## Orientation Handling

The goal yaw is converted to a quaternion using the standard 2D rotation formula:

```text
qx = 0
qy = 0
qz = sin(yaw / 2)
qw = cos(yaw / 2)
```

This produces a pure Z-axis rotation suitable for planar navigation.

## Frame Handling

The current frame strategy is:

1. The selected target arrives in `base_link` (the default output frame of the upstream pipeline)
2. The adapter attempts a TF transform from `base_link` to the configured `target_frame` (default `map`)
3. If `map` is available in the TF tree, the goal is published in `map`
4. If `map` is **not** available (no SLAM or Nav2 running), the adapter falls back to publishing in `base_link` and logs a warning

### Why this fallback is intentional

The current simulation setup does not always include SLAM or Nav2. The `map` frame only exists when a full navigation stack is running. The fallback behavior means:

- Stage 5 can be tested and validated without requiring the full Nav2 stack
- when the full stack is running, goals are automatically published in `map`
- no code changes are needed when switching between minimal and full setups

## Important Assumptions

This MVP depends on several practical assumptions:

- the input target position is already reasonably valid (from Stage 2 + 3 + 4)
- the target point is near the semantic object, not necessarily at its geometric center
- a safe stand-off distance is preferable to directly navigating into the localized point
- table/bench targets may correspond to a leg return rather than a semantic center
- the TF tree may or may not include `map` depending on the current launch configuration

## Validated Behavior

Current testing showed the expected adapter behavior:

- successful query results lead to `PoseStamped` goal output on `/nav_goal_adapter_node/goal_pose`
- generated goals are offset from the target by `approach_distance`
- goal orientation faces the target
- when `map` is unavailable, the goal is published in `base_link` with a logged warning
- Stage 5 has been verified as part of the end-to-end semantic pipeline

## Current Supported Semantic Test Objects

The current validated semantic test set is:

| semantic_name | detector_label | gazebo_model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

These names belong to different layers:

- `semantic_name` — human / NLP / project-level semantic name
- `detector_label` — exact label produced by YOLO
- `gazebo_model` — Gazebo model directory name

## Package Structure

```text
tb3_nav_adapter/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── nav_goal_adapter.yaml
├── launch/
│   └── nav_goal_adapter.launch.py
└── tb3_nav_adapter/
    ├── goal_adapter_core.py
    └── nav_goal_adapter_node.py
```

### File roles

- `tb3_nav_adapter/goal_adapter_core.py`
  - pure Python math for approach pose computation and yaw-to-quaternion conversion

- `tb3_nav_adapter/nav_goal_adapter_node.py`
  - ROS 2 node wrapper
  - subscribes to SemanticQueryResult
  - publishes PoseStamped
  - handles TF lookup with fallback

- `config/nav_goal_adapter.yaml`
  - runtime configuration for approach geometry, topics, and frame handling

- `launch/nav_goal_adapter.launch.py`
  - ROS 2 launch entry point

## Configuration

### `approach_distance`

- default: `0.5` m
- how far from the target the robot should stop

### `min_standoff_distance`

- default: `0.3` m
- reject targets closer than this to avoid collision

### `target_frame`

- default: `"map"`
- desired output frame for the PoseStamped goal
- falls back to source frame if TF is unavailable

### `tf_timeout`

- default: `0.5` s
- how long to wait for TF lookup before falling back

## How To Run

### 1. Launch the Gazebo test world

```bash
cd ~/TurtleBot3-semantic-navigation
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py
```

### 2. Launch `tb3_detector`

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true
```

### 3. Launch `tb3_localizer`

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true
```

### 4. Launch `semantic_memory_node`

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tb3_memory semantic_memory.launch.py use_sim_time:=true
```

### 5. Launch `semantic_query_node`

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tb3_query semantic_query.launch.py use_sim_time:=true
```

### 6. Launch `nav_goal_adapter_node`

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tb3_nav_adapter nav_goal_adapter.launch.py use_sim_time:=true
```

### 7. Send a command and inspect the goal

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 topic pub --once /semantic_query_node/command std_msgs/String "data: 'go to the person'"

ros2 topic echo /nav_goal_adapter_node/goal_pose
```

## Limitations

Current limitations are intentional and acceptable for the MVP:

- output is only a generated goal pose, not direct Nav2 execution
- table-like targets may have imperfect semantic center estimation
- no advanced obstacle-aware goal refinement yet
- current system depends on the quality of upstream localizer output
- base_link fallback is practical but not the final map-frame solution

## Future Work

Useful next steps include:

- connect PoseStamped output to Nav2 `NavigateToPose` action execution
- improve map-frame integration with full SLAM stack
- refine approach-point generation for table-like or elongated objects
- add object-type-specific approach policies
- integrate with full semantic navigation pipeline and frontier exploration

The current goal of `tb3_nav_adapter` is to provide a clean, validated bridge between semantic perception and downstream navigation execution.
