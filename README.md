# TurtleBot3 Gazebo Navigation Course

> **⚠️ REFERENCE BRANCH — do not hand this to students.** `detector_core.py`
> here contains the completed `load()` and `infer()`; the student version lives
> on `main`. Names, topics and configs are identical, so only the stub notices
> in this README and in `detector_node.py` no longer describe reality.

A ROS 2 course workspace for **object-based semantic navigation** on a simulated
TurtleBot3. Gazebo simulation, SLAM, Nav2, autonomous frontier exploration,
semantic memory, a rule-based command parser and a coordinator state machine are
all provided and working; the object detector in `tb3_detector` is the piece the
course asks you to write, and on this branch it is already complete, so the whole
pipeline runs end to end. The assignment, interface contract and acceptance
criteria are in **[INSTRUCTIONS.md](INSTRUCTIONS.md)**. Design rationale, measured
results and advanced debugging notes are in **[NOTES.md](NOTES.md)**.

## Demo

Two clips are planned and land in `docs/media/`:

- **`exploration.gif`** — autonomous frontier exploration mapping the room.
- **`navigate.gif`** — `go to person 0` interrupting exploration and driving to
  the target.

<!-- DEMO_PLACEHOLDER: docs/media/exploration.gif -->
<!-- DEMO_PLACEHOLDER: docs/media/navigate.gif -->

## System requirements

- Ubuntu 22.04 with **ROS 2 Humble** and **Gazebo Classic 11**
- A machine that can run Gazebo + Nav2 + SLAM comfortably (4+ CPU cores recommended)
- `tmux`, for `scripts/acceptance_run.sh` only

## Installation

### Ubuntu 22.04 (native)

**1. apt packages**

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
  python3-pip \
  tmux
```

**2. pip packages** (the detector imports them)

```bash
pip install 'ultralytics==8.4.31'
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

**3. Detector weights** — nothing to do. `src/tb3_detector/models/tb3det_yolo26n.pt`
(5.4 MB, yolo26n fine-tuned on the three course targets) ships with the
repository and is picked up by the build in step 4. The COCO `yolo26n.pt` it
replaced is optional, for the comparison experiment in NOTES.md:

```bash
# optional: COCO weights for the NOTES.md comparison (git-ignored)
wget -O src/tb3_detector/models/yolo26n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
```

**4. Build**

```bash
cd ~/turtlebot3-gazebo-navigation-course
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Two things to know:

- **If you have conda installed, run `conda deactivate` first** (until
  `(base)` disappears from your prompt).
- To rebuild a single package later:
  `colcon build --symlink-install --packages-select <pkg>` (e.g.
  `tb3_detector`). Keep `--symlink-install` on every build: the first build
  set it, and dropping it later copies files into `install/` that the
  symlinked build expects to be links, so edits to configs and launch files
  silently stop taking effect.

The Gazebo models used by the course worlds (person, trash can, chair, …)
are **vendored in this repository** (`src/tb3_bringup/models/`, next to the
worlds that reference them in `src/tb3_bringup/worlds/`; see
[NOTICE](NOTICE)) and wired up via `GAZEBO_MODEL_PATH` inside the launch
files — no online model database access is needed.

### macOS (Docker) — coming soon

Not available yet; the steps below mirror the Ubuntu skeleton above so the gaps
are visible. For now use a native or VM Ubuntu 22.04 installation.

- **1. Install Docker Desktop** — TBD
- **2. Pull the course image** — TBD
- **3. Detector weights** — TBD
- **4. Run the container with X11/GUI forwarding** — TBD

### Windows (Docker) — coming soon

Not available yet. For now use a native or VM Ubuntu 22.04 installation.

- **1. Install Docker Desktop + WSL2** — TBD
- **2. Pull the course image** — TBD
- **3. Detector weights** — TBD
- **4. Run the container with X11/GUI forwarding** — TBD

## Running — one subsystem per terminal

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
| T1 | `ros2 launch tb3_bringup sim.launch.py` | Gazebo (server + GUI) + TurtleBot3 spawn, vendored `GAZEBO_MODEL_PATH` | Console prints `Successfully spawned entity [waffle_pi]` and the Gazebo window shows the room + robot (~5 s; first-ever Gazebo start can take longer) |
| T2 | `ros2 launch tb3_bringup nav.launch.py` | SLAM Toolbox + Nav2 + RViz | Console prints `[lifecycle_manager_navigation]: Managed nodes are active` (~5–10 s); RViz shows a first gray map patch around the robot |
| T3 | `ros2 launch tb3_bringup backend.launch.py` | Course backend: memory, semantic map memory, query, nav adapter, coordinator, warmup + frontier exploration | `CoordinatorNode ready — mode=EXPLORING` appears immediately; the robot does a short ±45° warm-up scan, then `frontier exploration enabled` (~10 s) and the robot starts exploring |
| T4 | `ros2 launch tb3_bringup localizer.launch.py` | Localizer (bbox + LiDAR → object position) | `LocalizerNode ready` + `Image width learned: 640 px` (~1 s), then quiet until detections arrive |
| T5 | `ros2 launch tb3_bringup detector.launch.py` | Detector: YOLO inference on the camera image (shipped weights: `tb3det_yolo26n`, fine-tuned) | `Model loaded. Classes: [...]` then `detector_node ready` (~3 s, first inference warms up torch); `ros2 topic echo /detector_node/detections` streams non-empty `detections` once an object is in view; RViz **Detector Debug Image** shows green bounding boxes |
| T6 | *(no launch — the command console)* | Send user commands, watch status | — |

T6 commands:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 0'"
ros2 topic echo /coordinator_node/status
```

You can restart any single terminal without touching the others.

**Exploration takes 2–8 minutes.** Once T3 is up the robot drives itself
around the room until the frontier is exhausted, and only objects it has
actually seen can be navigated to — so a `go to …` command issued too early
fails with `no active <target> in memory`. The spread is normal: it depends
on the world, the spawn pose and how long Nav2 spends on recovery
behaviours. Watch `/semantic_memory_markers` in RViz and send commands once
the landmarks you want have appeared.

### Choosing a world

| `world:=` alias | File | Contents | Auto spawn |
|---|---|---|---|
| `warehouse_models_person` *(default)* | `warehouse_models_person.world` | 6×6 m room, 5 person figures (corners + centre) | `(-1.5, 0.0)` |
| `warehouse_models` | `warehouse_semantic_models.world` | 4×6 m room, 1 person + 1 trash can + 1 chair (all three semantic targets) | `(-1.2, -1.2)` |

The world is picked in T1 (everything else is world-agnostic):

```bash
ros2 launch tb3_bringup sim.launch.py world:=warehouse_models
```

An absolute path to a custom `.world` file is also accepted. The same
`world:=` argument works on the one-command launch below.

### Appendix: one-command launch

For demos and smoke tests:

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_bringup full_stack.launch.py
```

Give it ~30 s to settle. It forwards `world:=`, `use_rviz:=` and
`use_runtime_debug:=`. For day-to-day work use the six-terminal flow above.

For an unattended end-to-end acceptance run (clean restart, the six terminals in
tmux, the navigation commands, and a pass/fail report), run
[`scripts/acceptance_run.sh`](scripts/acceptance_run.sh). It needs `tmux`, and
defaults to `--world warehouse_models`, the world holding all three targets;
`--dry-run` prints the plan without starting anything.

## Where to change things

Four files cover almost every edit. Everything is under `src/`.

| I want to … | Edit | Notes |
|---|---|---|
| **Use a different world** | `src/tb3_bringup/worlds/*.world`, and `WORLD_PRESETS` in `src/tb3_bringup/launch/sim.launch.py` to give it a `world:=` alias and a spawn pose | Objects the world places must exist in `src/tb3_bringup/models/`; add a new prop there first |
| **Add or rename a semantic target** | `src/tb3_bringup/config/semantic_targets.yaml` | The one registry mapping `semantic_name` ↔ `detector_label` ↔ Gazebo model. Add the detector label to `class_filter` in `detector.yaml` too |
| **Tune detector thresholds** | `src/tb3_detector/config/detector.yaml` | `conf_threshold`, `class_filter`, `device`. `class_filter` takes detector labels (`"trash_can"`, or `"traffic light"` with the COCO weights), never semantic names |
| **Tune exploration / goal assignment** | `src/tb3_frontier_exploration/config/params.yaml` | Frontier size, goal spacing, blacklist TTL. The thresholds interlock — see NOTES.md |
| **Tune memory, query, approach pose** | `src/tb3_memory/config/`, `src/tb3_query/config/`, `src/tb3_nav_adapter/config/`, `src/tb3_coordinator/config/` | One YAML per package, named after the node it configures |
| **Change RViz layout** | `src/tb3_bringup/rviz/semantic_nav.rviz` | Opened by `nav.launch.py`; `use_rviz:=false` turns it off |
| **Change which nodes start** | `src/tb3_bringup/launch/` | One launch file per terminal; `src/tb3_bringup/launch/README.md` maps each to its terminal |
| **Change detection code** | `src/tb3_detector/tb3_detector/detector_core.py` | The graded file. See [INSTRUCTIONS.md](INSTRUCTIONS.md) |

**`build/`, `install/` and `log/` are generated — do not read them and do not
edit them.** `colcon build` writes all three from `src/`; `install/` holds
symlinks back into `src/` (that is what `--symlink-install` does), so an edit
made there is either overwritten on the next build or silently editing the
source through a link. Anything you actually want to keep goes in `src/`.
All three are git-ignored; deleting them and rebuilding is a safe reset.

## Troubleshooting

### Clean restart

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

### Robot spins on the spot and never drives

**Symptom.** The robot rotates in place indefinitely instead of exploring,
and T2 repeats:

```text
[controller_server] [ERROR] Failed to make progress
[controller_server] [WARN]  [follow_path] [ActionServer] Aborting handle.
```

Frontier goals keep being accepted, `/cmd_vel` carries a non-zero
`angular.z` with `linear.x` stuck at 0, and nothing is actually in the way —
`ros2 topic echo /scan` shows metres of clear space all round.

**This is not caused by your code.** It is an occasional wedge in the Nav2
controller/recovery loop; it happens with the reference detector too, and it
can occur before the detector has published anything at all. Nothing in
`detector_core.py` can cause it or fix it — the detector is not in the
control loop.

**Fix.** `Ctrl-C` every terminal and run the clean-restart procedure
above, then re-open T1–T5 in order. It clears on restart. The map and any
landmarks built so far are lost, so the robot re-explores from scratch.

Do **not** try to fix this by lowering `conf_threshold` or widening
`class_filter` — they are unrelated; see [NOTES.md](NOTES.md).

## Repository layout

| Package | Role |
|---|---|
| `tb3_bringup` | every launch file, the RViz config, the Gazebo worlds, the vendored models and `semantic_targets.yaml`. No code of its own |
| `tb3_detector` | YOLO detection on the camera image (`tb3det_yolo26n`, fine-tuned yolo26n; weights included) |
| `tb3_localizer` | bbox centre → bearing + LiDAR range → `(x, y)` in `base_link` |
| `tb3_memory` | short-term memory, stable IDs `person_0…` |
| `tb3_coordinator` | persistent map landmarks + the coordinator state machine |
| `tb3_query` | rule-based command parsing (`SemanticQueryResult` msg) |
| `tb3_nav_adapter` | approach-pose computation for Nav2 |
| `tb3_frontier_exploration` | frontier detection + goal assignment (C++) |

How they fit together:

```text
Gazebo camera ──► tb3_detector  (YOLO, 2D boxes)
                      │ /detector_node/detections
Gazebo LiDAR ───► tb3_localizer   (pixel bearing + LiDAR range → (x,y))
                      ▼
                  tb3_memory      (stable IDs person_0, person_1, …)
                      ▼
                  semantic_map_memory (persistent landmarks on SLAM map)
                      ▼
"go to person 2" ► tb3_query      (rule-based command parsing)
                      ▼
                  tb3_nav_adapter (standoff approach pose)
                      ▼
                  tb3_coordinator + Nav2 (pauses exploration, drives there)
```

## License

Course material is MIT-licensed (see [LICENSE](LICENSE)). The vendored
Gazebo models are third-party content — see [NOTICE](NOTICE) for
provenance and licensing status.
