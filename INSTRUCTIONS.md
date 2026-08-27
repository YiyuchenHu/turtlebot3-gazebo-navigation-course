# Assignment: Give the Robot Eyes — YOLOv8 Object Detection in ROS 2

## 1. Learning goals

By completing this assignment you will learn to:

1. **Integrate a pretrained deep-learning model (YOLOv8) into a ROS 2
   robot system** — managing Python dependencies, model weights, and
   inference inside a node that must keep up with a live camera stream.
2. **Program against a message contract**: your detector's output feeds a
   chain of downstream nodes you did not write. If your bounding boxes,
   labels, or coordinate conventions are wrong, the failure appears far
   away (the robot drives to the wrong place) — a very realistic robotics
   debugging experience.
3. **Understand a complete semantic-navigation pipeline**: detection →
   camera/LiDAR fusion → semantic memory → language-style commands →
   Nav2 goals, all running alongside SLAM and autonomous exploration.

The only file you must modify is
`src/tb3_detector/tb3_detector/detector_core.py`.
Everything else is provided and working.

## 2. Before you code: run the baseline

Build and launch as described in the [README](README.md) Quick start. Verify:

- the robot explores and maps the room on its own;
- the RViz **Detector Debug Image** panel shows the (box-free) camera stream;
- `ros2 topic echo /detector_node/detections` shows empty `detections: []`
  arrays streaming at camera rate;
- `ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 0'"`
  produces `query failed: no active person in memory` on
  `/coordinator_node/status` (watch with `ros2 topic echo`), and exploration
  resumes ~3 s later.

That last message is your "before" snapshot: the whole stack is idling,
waiting for detections that never come. Your job is to make them come.

## 3. Step-by-step tasks

### Step 1 — Install the Python dependencies

```bash
pip install 'ultralytics==8.4.31'   # pinned: yolo26n needs >=8.4.x;
                                    # the course is validated on 8.4.31
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

(CPU inference is sufficient; a GPU + `device: "cuda:0"` in
`config/detector.yaml` is optional.)

### Step 2 — Download the pretrained weights (~6 MB)

```bash
cd ~/turtlebot3-gazebo-navigation-course
wget -O src/tb3_detector/models/yolo26n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt
colcon build --packages-select tb3_detector   # copies the weights into install/
```

`*.pt` files are git-ignored on purpose: weights are downloaded, never
committed.

### Step 3 — Implement `DetectorCore`

Open `src/tb3_detector/tb3_detector/detector_core.py`. The two stubbed
methods contain detailed `TODO(student)` blocks:

- `load()` — import `ultralytics`, load `self.model_path`, move the model to
  `self.device`. Fail loudly (clear exception message) if the package or the
  weights file is missing.
- `infer(bgr_image)` — run prediction with `self.conf_threshold`, apply
  `self.class_filter`, and return a list of dicts in the **exact** format
  below.

The ROS wrapper (`detector_node.py`) needs no changes.

**Your daily development loop touches only Terminal 5** (see the README
six-terminal table). Everything else keeps running:

1. Edit `detector_core.py`.
2. In T5: `Ctrl-C`, then re-run the same
   `ros2 launch tb3_detector detector.launch.py`.
3. Watch T5's log and the RViz **Detector Debug Image**.

No rebuild is needed for Python edits: the workspace is built with
`--symlink-install`, so the installed detector code *is* your source
file. (Rebuilding is only needed when files are added/removed — e.g.
after downloading the weights in Step 2.)

**Don't touch the `.msg` files** (`src/tb3_query/msg/`). The
`SemanticQueryResult` message is compiled into three packages
(`tb3_query`, `tb3_nav_adapter`, `tb3_coordinator`); changing it forces
a full rebuild and breaks the grader's interface contract.

### Step 4 — Self-test the detector in isolation

```bash
# Terminal 1: static self-test world (person + marble table + stop sign in front).
# NOTE: this world predates the 2026-08-27 target change and still holds the two
# retired props. With the shipped class_filter only `person` will be reported;
# that is enough to prove your infer() works.
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py

# Terminal 2:
ros2 launch tb3_detector detector.launch.py

# Terminal 3:
ros2 topic echo /detector_node/detections
ros2 run rqt_image_view rqt_image_view /detector_node/debug_image
```

Expected: a box labelled `person` with a sensible confidence. If the debug
image shows boxes in the wrong place, re-read the pixel-coordinate
convention below.

### Step 5 — Full-stack test (default 5-person world)

Launch the full stack, let the robot explore until person markers appear in
RViz (`/semantic_memory_markers`), then:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person'"     # nearest
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 1'"   # by ID
```

IDs `person_0 … person_4` are assigned in **observation order** (they are
memory slots, not identities). `ros2 topic echo /semantic_map_memory_node/landmark_objects`
shows the live mapping.

### Step 6 — Trash can and chair (second world)

Restart the six-terminal flow, picking the second world in T1 (the other
terminals are world-agnostic and restart unchanged):

```bash
ros2 launch tb3_coordinator sim.launch.py world:=warehouse_models
```

This world holds all three targets — person, trash can, chair — so all three
commands should work once your `infer()` is correct:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to trash can'"
ros2 topic pub --once /user_command std_msgs/String "data: 'go to chair'"
```

`go to trash can` is the one to think about. COCO has **no trash-can class**,
so the detector reports this model as **`traffic light`**, and
`semantic_targets.yaml` maps `traffic light` → `trash_can`. Your `infer()`
must report the raw COCO label; if you "helpfully" rename it, the mapping
breaks. That mapping is the whole reason `semantic_name` and `detector_label`
are separate fields.

Two targets were retired on 2026-08-27 after measurement, and it is worth
knowing why — both failures were in the **LiDAR**, not the detector:

| retired | why |
|---|---|
| `table` (`table_marble`) | its link is posed at `z=0.648`, so the geometry sits above the 0.121 m scan plane: 6 of 9 test poses returned no LiDAR range at all |
| `stop_sign` | the pole is too thin — only 16% of the beams in the ±5° window hit it, so the range came from the wall behind and the landmark landed 4.38 m from the real sign |

The lesson generalises: an object is only usable as a landmark if it is
**both** recognisable to the camera **and** solid to the LiDAR between the
floor and ~0.3 m.

## 4. Interface contract (what the grader's stack assumes)

### Topics (names must not change)

| Direction | Topic | Type | QoS |
|---|---|---|---|
| in | `/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT (Gazebo sensor) |
| in | `/camera/camera_info` | `sensor_msgs/CameraInfo` | BEST_EFFORT |
| out | `/detector_node/detections` | `vision_msgs/Detection2DArray` | RELIABLE |
| out | `/detector_node/debug_image` | `sensor_msgs/Image` | RELIABLE |

### `DetectorCore.infer()` return format (consumed by `detector_node.py`)

```python
[
  {
    "label":     "person",          # raw COCO class name from YOLO
    "conf":      0.87,              # float 0..1
    "bbox_xyxy": [x1, y1, x2, y2],  # float pixels, ORIGINAL image resolution
    "track_id":  None,              # int only if you enable tracking
  },
  ...
]
```

### Pixel-coordinate convention

- Origin `(0, 0)` = **top-left** of the image; x → right, y → down.
- `(x1, y1)` = top-left corner of the box, `(x2, y2)` = bottom-right.
- Coordinates are in pixels of the **original** `/camera/image_raw`
  resolution. If you resize before inference, scale the boxes back
  (ultralytics already returns original-resolution boxes — don't touch them).
- Downstream, the localizer maps the bbox **centre x** linearly onto the
  camera's 62.2° horizontal FOV to get a bearing, then reads a LiDAR range
  window at that bearing. A horizontally shifted box = a wrong bearing = the
  robot localises the object in the wrong direction.

### Labels and thresholds

- Report **raw COCO labels** (`"person"`, `"traffic light"`, `"chair"`). The
  task-level names (`person`, `trash_can`, `chair`) are resolved downstream
  via `semantic_targets.yaml`. Note that the trash can is detected as
  **`traffic light`** — COCO has no trash-can class.
- The shipped `conf_threshold: 0.35` was set from a full-stack run. Below it,
  yolo26n occasionally calls something near the robot a `traffic light` at
  ~0.30 and plants a phantom trash-can landmark. True positives sit well
  above: trash can 0.31-0.65, chair p50 0.87, person p50 ~0.9. You may tune it
  in `config/detector.yaml`, but the acceptance test runs with the shipped config.

## 5. Acceptance criteria

Your implementation passes when, **with your `detector_core.py` as the only
code change**:

1. `colcon build --symlink-install` succeeds and the stack runs without
   node crashes — both in the six-terminal flow and via the one-command
   `full_semantic_nav.launch.py` (README appendix), which is what the
   grader's batch run uses.
2. In the default world, after exploration has seen the room:
   `go to person` and `go to person N` (for at least two different observed
   `N`) each end with the coordinator reporting `TARGET_REACHED` and the
   robot physically stopped ~0.5 m from the correct person, then exploration
   resumes automatically.
3. In `world:=warehouse_models`: `go to trash can` and `go to chair` both
   reach their targets the same way. "Reaches" means the robot ends up
   within 1.0 m of the real object — a `TARGET_REACHED` on its own is not
   enough, because Nav2 will happily report success at a mislocalised
   landmark.
4. RViz shows your detections live: boxes in the **Detector Debug Image**
   panel and sphere+text landmarks (`/semantic_memory_markers`) on the map.
5. No interface drift: topic names, message types, and the
   `infer()` dict format are unchanged.

## 6. Debugging advice

- **Look at the debug image first** (`rqt_image_view /detector_node/debug_image`).
  No boxes there = detection problem. Boxes there but no landmarks in RViz =
  check the chain *after* the detector, in order.
- **Walk the pipeline with `ros2 topic echo`**:
  `/detector_node/detections` → `/localizer_node/localized_objects` →
  `/semantic_memory_node/objects` → `/semantic_map_memory_node/landmark_objects`.
  The first silent topic tells you which stage lost your object.
- **Runtime statistics overlay**: start T3 with
  `ros2 launch tb3_coordinator course_backend.launch.py use_runtime_debug:=true`
  to add `semantic_runtime_debug_node`, which logs per-stage counts and
  per-class confusion diagnostics (CSV under `/tmp/semantic_debug`).
- **Common pitfalls**
  - `class_filter` entries are COCO **detector labels**: `"traffic light"`,
    not `"trash_can"`. Multi-word COCO labels contain a space.
  - Never add `"airplane"` to `class_filter`. yolo26n fires it on many
    untextured Gazebo props at once, so it cannot identify any single
    target and will poison your landmarks.
  - Never write a bare `[]` for `class_filter` in YAML (rclpy Humble
    type-inference crash) — use `[""]` to mean "all classes".
  - Weights added *after* building are not in `install/` until you re-run
    `colcon build --packages-select tb3_detector` (the launch resolves
    `model_path` from the install tree).
  - The camera publishes BEST_EFFORT — if you create your own image
    subscriptions, RELIABLE QoS will silently receive nothing.
  - Confidence too high → the chair (weakest of the three head-on) never
    appears; too low → ghost landmarks. Start from the shipped 0.35.
  - Every course launch (T1–T5 and the one-command shell) already
    defaults to `use_sim_time:=true` — no flag needed in simulation.
    Only pass `use_sim_time:=false` if you reuse a node on a real robot.

## 7. Bonus (optional): rewrite the localizer

For extra credit, re-implement `tb3_localizer`'s node
(`localizer_node.py` / `localizer_core.py`) from scratch against its
interface: consume your `/detector_node/detections` + `/scan` +
`/camera/image_raw` (width only), and publish
`vision_msgs/Detection3DArray` on `/localizer_node/localized_objects` with
per-object `(x, y)` in `base_link` (bbox centre → bearing through the
62.2° HFOV → windowed-median LiDAR range → planar projection). The provided
implementation is your reference and test oracle: swap yours in and the
rest of the stack should behave identically.

While you are there, look at the **known frame simplification** in
`tb3_nav_adapter` (see README "Known simplifications"): explain in one
paragraph what `compute_approach_pose` assumes about the target's
coordinate frame, why it still works in these worlds, and how you would fix
it properly (hint: TF lookup of `base_link` in `map` at query time).
