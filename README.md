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

```bash
cd ~/turtlebot3-gazebo-navigation-course
./build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_coordinator full_semantic_nav.launch.py
```

The launch brings up (with staggered start-up timers) Gazebo + TurtleBot3,
Nav2 with SLAM Toolbox, RViz, the perception chain, frontier exploration,
and the coordinator. Give it ~30 s to settle on first start.

Send commands from a second terminal (same workspace sourced):

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 0'"
```

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

```bash
ros2 launch tb3_coordinator full_semantic_nav.launch.py world:=warehouse_models
```

An absolute path to a custom `.world` file is also accepted.

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

## License

Course material is MIT-licensed (see [LICENSE](LICENSE)). The vendored
Gazebo models are third-party content — see [NOTICE](NOTICE) for
provenance and licensing status.
