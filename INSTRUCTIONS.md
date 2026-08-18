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
pip install ultralytics
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

(CPU inference is sufficient; a GPU + `device: "cuda:0"` in
`config/detector.yaml` is optional.)

### Step 2 — Download the pretrained weights (~6 MB)

```bash
cd ~/turtlebot3-gazebo-navigation-course
wget -O src/tb3_detector/models/yolov8n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
./build.sh   # re-run so the weights are copied into install/
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

### Step 4 — Self-test the detector in isolation

```bash
# Terminal 1: static test world (person + table + stop sign right in front)
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py

# Terminal 2:
ros2 launch tb3_detector detector.launch.py use_sim_time:=true

# Terminal 3:
ros2 topic echo /detector_node/detections
ros2 run rqt_image_view rqt_image_view /detector_node/debug_image
```

Expected: boxes labelled `person` and `bench` (the marble table!) with
sensible confidences; `stop sign` too if you temporarily widen
`class_filter` (see Step 6). If the debug image shows boxes in the wrong
place, re-read the pixel-coordinate convention below.

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

### Step 6 — Table and stop sign (second world)

```bash
ros2 launch tb3_coordinator full_semantic_nav.launch.py world:=warehouse_models
```

- `go to table` (or `…the bench`) should work immediately — YOLO reports the
  marble table as COCO class `bench`, and the provided mapping
  (`src/tb3_frontier_exploration/config/semantic_targets.yaml`) translates it.
- The stop sign ships disabled at every layer (a deliberate exercise). To
  make `go to stop sign` work you must re-enable all three:
  1. **World**: in `src/tb3_frontier_exploration/worlds/warehouse_semantic_models.world`,
     uncomment the `<include>` block for `model://stop_sign`.
  2. **Target registry**: set `enabled: true` for `stop_sign` in
     `semantic_targets.yaml`.
  3. **Detector filter**: add `"stop sign"` (with the space!) to
     `class_filter` in `src/tb3_detector/config/detector.yaml`.
  Rebuild (`./build.sh`), relaunch, and try again. This three-file change is
  part of the assignment — it proves you understand the naming layers
  (gazebo_model vs semantic_name vs detector_label).

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

- Report **raw COCO labels** (`"person"`, `"bench"`, `"stop sign"`). The
  task-level names (`person`, `table`, `stop_sign`) are resolved downstream
  via `semantic_targets.yaml`. In this simulation the marble table is
  detected as **`bench`** (COCO 13) — *not* `"dining table"`.
- The shipped `conf_threshold: 0.12` is deliberately low (validated for this
  Gazebo scene, where the table is a borderline `bench`). Typical values are
  0.25–0.5; you may tune it in `config/detector.yaml`, but the acceptance
  test runs with the shipped config.

## 5. Acceptance criteria

Your implementation passes when, **with your `detector_core.py` as the only
code change** (plus the three documented stop-sign edits from Step 6):

1. `colcon build` succeeds and `full_semantic_nav.launch.py` runs without
   node crashes.
2. In the default world, after exploration has seen the room:
   `go to person` and `go to person N` (for at least two different observed
   `N`) each end with the coordinator reporting `TARGET_REACHED` and the
   robot physically stopped ~0.5 m from the correct person, then exploration
   resumes automatically.
3. In `world:=warehouse_models`: `go to table` and (after Step 6's config
   change) `go to stop sign` both reach their targets the same way.
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
- **Runtime statistics overlay**: launch with `use_runtime_debug:=true` to
  start `semantic_runtime_debug_node`, which logs per-stage counts and
  person/bench confusion diagnostics (CSV under `/tmp/semantic_debug`).
- **Common pitfalls**
  - `class_filter` entries are COCO **detector labels**: `"bench"`, not
    `"table"`; `"stop sign"` with a space, not `"stop_sign"`.
  - Never write a bare `[]` for `class_filter` in YAML (rclpy Humble
    type-inference crash) — use `[""]` to mean "all classes".
  - Weights added *after* building are not in `install/` until you re-run
    `./build.sh` (the launch resolves `model_path` from the install tree).
  - The camera publishes BEST_EFFORT — if you create your own image
    subscriptions, RELIABLE QoS will silently receive nothing.
  - Confidence too high → the marble table (`bench`, weak detection) never
    appears; too low → ghost landmarks. Start from the shipped 0.12.
  - Keep `use_sim_time:=true` everywhere (launch files already do this).

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
