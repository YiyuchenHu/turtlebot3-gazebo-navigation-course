# `tb3_detector` — YOUR ASSIGNMENT PACKAGE

> **⚠️ REFERENCE BRANCH — the detector is fully implemented here.**
> The text below describes the student-facing package on `main`, where
> `detector_core.py` is a stub. On this branch `load()` and `infer()` are
> complete and the node publishes real detections.

`tb3_detector` is Stage 1 of the TurtleBot3 perception pipeline. It answers
the first perception question:

> What does the robot see?

**In this course repository the YOLO inference is deliberately removed.**
The package builds and runs as a *stub*: the node subscribes to the camera,
publishes an **empty** `vision_msgs/Detection2DArray` on
`/detector_node/detections`, and forwards the raw camera frame on
`/detector_node/debug_image` (so the RViz image panel works out of the box).
Every other package in the repository is provided fully working and consumes
these two topics.

**Your task: implement `tb3_detector/detector_core.py` (`load()` and
`infer()`) so the stack detects real objects. Full assignment text, interface
contract, and acceptance criteria: [`INSTRUCTIONS.md`](../../INSTRUCTIONS.md)
at the repository root.**

## What the finished node does

1. receives a camera image from `/camera/image_raw`
2. converts the ROS image message into an OpenCV BGR image (provided)
3. runs YOLOv8 inference (**you implement this**)
4. optionally filters detections by configured class labels (**you implement this**)
5. publishes a `vision_msgs/Detection2DArray` (provided)
6. publishes a debug image with bounding boxes (provided)

This stage is intentionally 2D-only: it detects objects in image space.
Geometry (pixel → bearing → position) is handled downstream by the provided
`tb3_localizer`.

## Input topics

| Topic | Type | Notes |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | main RGB stream (BEST_EFFORT QoS) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | cached only; not needed for 2D detection |

## Output topics

| Topic | Type | Notes |
|---|---|---|
| `/detector_node/detections` | `vision_msgs/Detection2DArray` | machine-readable detections (empty until implemented) |
| `/detector_node/debug_image` | `sensor_msgs/Image` | camera image + boxes; raw image while stubbed |

Each `Detection2D` carries:

- `results[0].hypothesis.class_id` → YOLO class label string
- `results[0].hypothesis.score` → confidence
- `bbox.center.position.x / y` → box centre in pixels
- `bbox.size_x / size_y` → box size in pixels
- `id` → tracking id if tracking is enabled, otherwise empty

The Stage-2 localizer uses the bbox **centre x** to estimate object bearing —
pixel coordinates must be in the original image resolution.

## The three naming layers (read this twice)

| semantic_name | detector_label (COCO) | gazebo_model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

`detector.yaml`'s `class_filter` must use **detector labels** (`"bench"`,
`"stop sign"`, `"person"`) — never semantic names. If this rule is broken the
detector runs fine but silently filters out everything you care about.
Canonical mapping: `src/tb3_frontier_exploration/config/semantic_targets.yaml`.

## Package structure

```text
tb3_detector/
├── package.xml
├── setup.py
├── config/
│   └── detector.yaml        ← tuning (conf_threshold, class_filter, topics)
├── launch/
│   └── detector.launch.py   ← resolves model_path, starts the node
├── models/
│   └── (yolov8n.pt — you download this; git-ignored)
└── tb3_detector/
    ├── detector_core.py     ← ★ YOUR CODE GOES HERE ★
    └── detector_node.py     ← provided ROS wrapper (no changes needed)
```

## Dependencies (once you start implementing)

```bash
sudo apt install ros-humble-vision-msgs ros-humble-cv-bridge
pip install ultralytics
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

Weights (~6 MB, git-ignored):

```bash
wget -O src/tb3_detector/models/yolov8n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
```

## Standalone testing (without the full stack)

```bash
# Terminal 1 — static test world with all three objects in front of the robot
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py

# Terminal 2 — just the detector
ros2 launch tb3_detector detector.launch.py use_sim_time:=true

# Terminal 3 — inspect
ros2 topic echo /detector_node/detections
ros2 run rqt_image_view rqt_image_view /detector_node/debug_image
```
