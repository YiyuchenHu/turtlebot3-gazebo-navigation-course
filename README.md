> **⚠️ OUTDATED BRANCH — do not work here.**
> `main` is frozen at 2026-08-18 and kept only as a historical reference.
> Current work lives on the `reference` branch: `git checkout reference`
> Since then the detector weights and model, the target objects, the launch
> procedure, and the directory layout have all changed — this README does
> not describe the current project.

# TurtleBot3 Gazebo Navigation Course

A ROS 2 course workspace for **object-based semantic navigation** on a
simulated TurtleBot3. The full infrastructure is provided and working:
Gazebo simulation, SLAM, Nav2, autonomous frontier exploration, semantic
memory, a rule-based command parser, and a coordinator state machine.

**One piece is intentionally missing: the object detector.**
`tb3_detector` ships as a stub that publishes *empty* detections. Your
assignment is to implement YOLOv8 inference inside it so that commands like
`go to person 2` actually drive the robot to a person.

➡ **The assignment, interface contract, and acceptance criteria are in
[INSTRUCTIONS.md](INSTRUCTIONS.md). Start there after the Quick start below.**

```text
Gazebo camera ──► tb3_detector (YOUR TASK: YOLOv8, 2D boxes)
                      │ /detector_node/detections
Gazebo LiDAR ───► tb3_localizer   (provided: pixel bearing + LiDAR range → (x,y))
                      ▼
                  tb3_memory      (provided: stable IDs person_0, person_1, …)
                      ▼
                  semantic_map_memory (provided: persistent landmarks on SLAM map)
                      ▼
"go to person 2" ► tb3_query      (provided: rule-based command parsing)
                      ▼
                  tb3_nav_adapter (provided: standoff approach pose)
                      ▼
                  tb3_coordinator + Nav2 (provided: pauses exploration, drives there)
```

## System requirements

- Ubuntu 22.04 with **ROS 2 Humble** and **Gazebo Classic 11**
- A machine that can run Gazebo + Nav2 + SLAM comfortably (4+ CPU cores recommended)
- **Docker support for macOS: in progress** — for now a native (or VM) Ubuntu
  22.04 installation is required.

### apt packages

```bash
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-vision-msgs ros-humble-cv-bridge \
  ros-humble-rqt-image-view \
  python3-colcon-common-extensions \
  python3-pip
```

### pip packages (only needed once you implement the detector)

```bash
pip install ultralytics
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

The Gazebo models used by the course worlds (person, marble table, stop
sign, …) are **vendored in this repository** (`src/tb3_frontier_exploration/models/`,
see [NOTICE](NOTICE)) and wired up via `GAZEBO_MODEL_PATH` inside the launch
files — no online model database access is needed.

## Quick start

### 1. Build (once)

```bash
cd ~/turtlebot3-gazebo-navigation-course
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Two things to know:

- **If you have conda installed, run `conda deactivate` first** (until
  `(base)` disappears from your prompt). A conda Python shadowing
  `/usr/bin/python3` breaks colcon and message generation in
  hard-to-diagnose ways.
- To rebuild a single package later:
  `colcon build --packages-select <pkg>` (e.g. `tb3_detector`).

### 2. Run — one subsystem per terminal

In **every** terminal, prepare the environment first:

```bash
cd ~/turtlebot3-gazebo-navigation-course
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

Then open the terminals **in order**, waiting for each ready signal
before starting the next:

| # | Command | What it starts | Ready when … |
|---|---|---|---|
| T1 | `ros2 launch tb3_coordinator sim.launch.py` | Gazebo (server + GUI) + TurtleBot3 spawn, vendored `GAZEBO_MODEL_PATH` | Console prints `Successfully spawned entity [waffle_pi]` and the Gazebo window shows the room + robot (~5 s; first-ever Gazebo start can take longer) |
| T2 | `ros2 launch tb3_coordinator nav.launch.py` | SLAM Toolbox + Nav2 + RViz | Console prints `[lifecycle_manager_navigation]: Managed nodes are active` (~5–10 s); RViz shows a first gray map patch around the robot |
| T3 | `ros2 launch tb3_coordinator course_backend.launch.py` | Course backend: memory, semantic map memory, query, nav adapter, coordinator, warmup + frontier exploration | `CoordinatorNode ready — mode=EXPLORING` appears immediately; the robot does a short ±45° warm-up scan, then `frontier exploration enabled` (~10 s) and the robot starts exploring |
| T4 | `ros2 launch tb3_localizer localizer.launch.py` | Localizer (bbox + LiDAR → object position) — bonus unit, yours to rewrite | `LocalizerNode ready` + `Image width learned: 640 px` (~1 s), then quiet until detections arrive |
| T5 | `ros2 launch tb3_detector detector.launch.py` | **The detector — your assignment.** Stub publishes empty detections until you implement it | `detector_node ready` after a loud STUB warning (~1 s); `ros2 topic echo /detector_node/detections` streams empty `detections: []`; RViz **Detector Debug Image** shows the camera stream |
| T6 | *(no launch — your command console)* | Send user commands, watch status | — |

T6 commands:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 0'"
ros2 topic echo /coordinator_node/status
```

Each launch is standalone: starting one without its upstream neighbours
never crashes — nodes simply wait for data. You can kill and restart any
single terminal (typically T5 while iterating on the detector) without
touching the others.

### Troubleshooting: clean restart

`Ctrl-C` normally shuts a terminal's launch down cleanly, but a killed
terminal or a crashed Gazebo can leave orphan processes behind. Symptoms
and the matching fix, in order:

1. **T1 won't start / `spawn_entity` hangs on `Waiting for service
   /spawn_entity` / Gazebo complains the master port is in use** — an old
   Gazebo is still running:
   ```bash
   pkill -9 -x gzserver; pkill -9 -x gzclient
   ```
2. **`ros2 node list` shows nodes you did not start (or duplicates), the
   robot chases exploration goals with everything closed** — orphan
   course/Nav2 nodes survive:
   ```bash
   pkill -9 -f 'detector_node|localizer_node|coordinator_node|semantic_memory_node|semantic_map_memory_node|semantic_query_node|nav_goal_adapter_node|frontier_detection_node|goal_assignment_node|startup_map_warmup'
   pkill -9 -f 'slam_toolbox|nav2|lifecycle_manager|bt_navigator|controller_server|planner_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother|map_saver'
   pkill -9 -x rviz2; pkill -9 -x robot_state_publisher
   ```
3. **Ghost topics/nodes still listed after everything is dead** — stale
   discovery cache:
   ```bash
   ros2 daemon stop        # restarts automatically on the next ros2 command
   ```

Then re-open T1–T5 in order as above. Verify the slate is clean with
`ros2 node list` (should be empty or error out).

## What works out of the box (before you write any code)

- The robot performs a warm-up rotation, then **explores and maps the room
  autonomously** (frontier exploration + Nav2 + SLAM).
- RViz shows the growing map, frontier markers, costmaps, and the
  **Detector Debug Image** panel — which displays the raw camera stream,
  because the stub detector forwards it without boxes.
- Object commands are accepted but **fail gracefully**: the stub publishes
  empty detections, so semantic memory stays empty and
  `go to person 0` answers `query failed: no active person in memory`
  on `/coordinator_node/status`, after which exploration resumes
  automatically. Once your detector works, the same command drives the
  robot to the person.

## Choosing a world

| `world:=` alias | File | Contents | Auto spawn |
|---|---|---|---|
| `warehouse_models_person` *(default)* | `warehouse_models_person.world` | 6×6 m room, 5 person figures (corners + centre) | `(-1.5, 0.0)` |
| `warehouse_models` | `warehouse_semantic_models.world` | 4×6 m room, 1 marble table + 1 person + 1 stop sign | `(-1.2, -1.2)` |

The world is picked in T1 (everything else is world-agnostic):

```bash
ros2 launch tb3_coordinator sim.launch.py world:=warehouse_models
```

An absolute path to a custom `.world` file is also accepted. The same
`world:=` argument works on the appendix one-command launch below.

## Repository layout

| Package | Status | Role |
|---|---|---|
| `tb3_detector` | **★ assignment (stub)** | YOLOv8 detection on the camera image |
| `tb3_localizer` | provided | bbox centre → bearing + LiDAR range → `(x, y)` in `base_link` |
| `tb3_memory` | provided | short-term memory, stable IDs `person_0…` |
| `tb3_coordinator` | provided | persistent map landmarks, state machine, RViz config, main launch |
| `tb3_query` | provided | rule-based command parsing (`SemanticQueryResult` msg) |
| `tb3_nav_adapter` | provided | approach-pose computation for Nav2 |
| `tb3_frontier_exploration` | provided | frontier detection + goal assignment (C++), worlds, vendored models |

## Known simplifications (intentional, documented)

- **`tb3_nav_adapter` frame assumption**: `compute_approach_pose` treats the
  target coordinates as if they were robot-relative (robot at the origin),
  but the configured pipeline feeds it **map-frame** landmarks. The 0.5 m
  standoff is therefore computed along the *map-origin→target* direction
  rather than *robot→target*. In these small rooms the map origin equals the
  spawn pose, so the error is modest and Nav2 still reaches the target —
  but it is a real simplification worth understanding (and a good discussion
  point; see the bonus task in INSTRUCTIONS.md).
- `tb3_frontier_exploration/config/params.yaml` names an
  `odom_topic: /odometry/filtered` that does not exist in this stack; the
  exploration nodes actually obtain the robot pose via TF, so the setting is
  inert. Don't let it mislead you.
- Landmark IDs (`person_0`, `person_1`, …) are assigned in **observation
  order** by semantic memory — they are memory slots, not person identities,
  and the same physical figure can receive a different index across runs.

## Appendix: one-command launch

For demos and smoke tests there is a convenience shell that includes the
five sub-launches with fixed start-up delays standing in for the
"wait until ready" steps you perform by hand in the six-terminal flow:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_coordinator full_semantic_nav.launch.py
```

Give it ~30 s to settle. It forwards `world:=`, `use_rviz:=` and
`use_runtime_debug:=`. Everything then shares one terminal's log stream —
fine for a demo, noisy for development. **For day-to-day work use the
six-terminal flow above**, which gives each subsystem its own logs and
lets you restart the detector alone.

## License

Course material is MIT-licensed (see [LICENSE](LICENSE)). The vendored
Gazebo models are third-party content — see [NOTICE](NOTICE) for
provenance and licensing status.
