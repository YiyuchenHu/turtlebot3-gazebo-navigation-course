# `tb3_localizer`

`tb3_localizer` is Stage 2 of the TurtleBot3 perception pipeline.

Stage 1 (`tb3_detector`) answers: "What does the robot see?"

Stage 2 (`tb3_localizer`) answers: "Where is the detected object near the robot?"

The current implementation is a practical MVP. It fuses:

- 2D detections from `/detector_node/detections`
- 2D LiDAR ranges from `/scan`
- Image width from `/camera/image_raw`

and publishes planar object points in `base_link`.

## What It Does

For each detected object, `tb3_localizer`:

1. Reads the bounding-box center x pixel from `vision_msgs/Detection2DArray`
2. Converts that pixel position into a horizontal bearing angle
3. Maps the bearing to the matching `LaserScan` direction
4. Reads a robust LiDAR range around that direction
5. Converts `(range, bearing)` into `(x, y)` in `base_link`
6. Publishes the result as `geometry_msgs/PointStamped`

In short, it estimates a 2D object position relative to the robot.

## What It Does Not Do Yet

This MVP does not do the following yet:

- No full 3D pose estimation
- No RGB-D depth projection
- No semantic memory
- No query logic
- No navigation goal generation
- No map-frame transform inside the node yet
- No multi-frame tracking or data association beyond the detector output

Future work can add TF-based `base_link -> map` conversion and semantic-memory integration on top of this output.

## Why This Method Is Used

The default TurtleBot3 Gazebo models used in this project do not provide a real depth-camera topic for RGB-D projection. Because of that, this project uses camera + 2D LiDAR fusion instead of RGB-D depth lookup.

This is a practical MVP because:

- the detector already provides reliable 2D bounding boxes
- TurtleBot3 already provides `/scan`
- planar object localisation is sufficient for the next stage
- it is simple to debug and easy to validate in simulation

This method is intentionally lightweight: it gives a usable robot-relative object position without introducing a more complex RGB-D or full 3D pipeline.

## Runtime Interfaces

### Inputs

- `/detector_node/detections`
  - Type: `vision_msgs/msg/Detection2DArray`
  - Source: `tb3_detector`

- `/scan`
  - Type: `sensor_msgs/msg/LaserScan`
  - Source: TurtleBot3 LiDAR

- `/camera/image_raw`
  - Type: `sensor_msgs/msg/Image`
  - Used only to learn image width for pixel-to-bearing conversion

### Output

- `/localizer_node/object_points`
  - Type: `geometry_msgs/msg/PointStamped`
  - One message is published per localized detection
  - `header.frame_id` is currently `base_link`

The current node publishes robot-relative points first. Transforming to `map` is left as future work.

## Localization Principle

The localizer uses the horizontal position of the detection in the image to infer a bearing, then fuses that bearing with LiDAR.

### 1. Bounding-box center x

For each `Detection2D`, the node reads:

- `u = bbox.center.position.x`
- `W = image width`

### 2. Pixel x to horizontal bearing

Let:

- `u` = bbox center x pixel
- `W` = image width
- `hfov` = camera horizontal field of view in radians

The localizer computes:

```text
normalized = (u - W/2) / (W/2)
bearing = -normalized * (hfov / 2)
```

Interpretation:

- `u = W/2`  -> `bearing = 0` -> object is straight ahead
- `u < W/2`  -> `bearing > 0` -> object is on the left
- `u > W/2`  -> `bearing < 0` -> object is on the right

### 3. Bearing to LaserScan direction

The current TurtleBot3 runtime uses a scan convention of approximately:

- `angle_min = 0.0`
- `angle_max ≈ 2π`
- `angle_increment ≈ 0.01749`

So the scan is indexed in `[0, 2π)`, not `[-π, π]`.

The node maps the camera bearing into scan angle space with:

```text
scan_angle = bearing mod (2π)
scan_index = round((scan_angle - angle_min) / angle_increment)
```

This correctly handles right-side bearings such as `-0.3 rad`, which should map to a scan angle near `2π - 0.3`.

### 4. Robust range near the matched direction

Instead of trusting one LiDAR ray, the node looks at a small local scan window around the central index:

```text
[center_index - scan_window_half, center_index + scan_window_half]
```

It discards:

- `nan`
- `inf`
- ranges smaller than `min_valid_range`
- ranges larger than `max_valid_range`

Then it uses the median of the remaining values as the object range.

This makes the result more stable than using a single ray directly.

### 5. Range + bearing to Cartesian point

With:

- `r = localized range`
- `theta = bearing`

the node computes:

```text
x = r * cos(theta)
y = r * sin(theta)
```

and publishes:

- `x` forward
- `y` left
- `z = 0`

## Frame and Sign Convention

The current output uses ROS REP 103 body-frame convention:

- `+x` forward
- `+y` left
- `+z` up

This means:

- left-side objects should produce `y > 0`
- center objects should produce `y ≈ 0`
- right-side objects should produce `y < 0`

## Important Assumptions

This MVP depends on several practical assumptions:

- The camera forward axis is roughly aligned with robot forward direction
- The image center corresponds approximately to robot forward
- The target object produces a usable LiDAR return near the same bearing
- Localisation is planar only
- The LiDAR return belongs to the detected object, not background clutter

One important caveat:

- thin, narrow, or visually obvious objects may still be difficult for LiDAR to localize if they do not intersect the scan plane well

This is especially relevant for signs, poles, and partially occluded objects.

## Validated Behavior

Current simulation testing showed the expected sign behavior:

- left-side objects -> `y > 0`
- center objects -> `y ≈ 0`
- right-side objects -> `y < 0`

It also showed an important limitation:

- the stop sign only localizes when it falls inside useful LiDAR range and the sign geometry intersects the scan plane well enough

So a detector hit does not always guarantee a valid LiDAR-based localization.

## Current Supported Semantic Test Objects

The current validated semantic test set is:

| semantic target | detector label | Gazebo model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop sign` | `stop sign` | `stop_sign` |

Notes:

- The project currently uses `stop sign` as the semantic target string in config files.
- The Gazebo model directory is `stop_sign`.
- The table is intentionally mapped to YOLO label `bench`, because that is what the current model detects reliably in simulation.

## Package Structure

```text
tb3_localizer/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── localizer.yaml
├── launch/
│   └── localizer.launch.py
└── tb3_localizer/
    ├── localizer_core.py
    └── localizer_node.py
```

### File roles

- `tb3_localizer/localizer_core.py`
  - Pure math and fusion logic
  - Pixel -> bearing -> scan index -> robust range -> `(x, y)`

- `tb3_localizer/localizer_node.py`
  - ROS 2 node wrapper
  - Subscribes to detections, scan, and image
  - Publishes `PointStamped`

- `config/localizer.yaml`
  - Node parameters such as camera HFOV, scan window size, and topics

- `launch/localizer.launch.py`
  - Simple ROS 2 launch entry point

## How To Run

### 1. Launch the Gazebo test world

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py
```

### 2. Launch the detector node

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true
```

### 3. Launch the localizer node

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true
```

### 4. Inspect localized object points

```bash
source /opt/ros/humble/setup.bash
source ~/TurtleBot3-semantic-navigation/install/setup.bash
ros2 topic echo /localizer_node/object_points
```

Optional:

```bash
ros2 topic hz /localizer_node/object_points
```

## Practical Debug Checklist

- Confirm detections are arriving:
  - `ros2 topic hz /detector_node/detections`

- Confirm LiDAR is arriving:
  - `ros2 topic hz /scan`

- Confirm image is arriving:
  - `ros2 topic hz /camera/image_raw`

- Check that the localizer learns image width:
  - localizer log should print `Image width learned: ... px`

- Check that localized points are being published:
  - `ros2 topic echo /localizer_node/object_points`

- Check sign behavior:
  - object on left -> positive `y`
  - object in center -> `y` near zero
  - object on right -> negative `y`

## Limitations and Future Work

The current MVP is intentionally simple. Useful next steps include:

- transform localized points from `base_link` to `map`
- integrate outputs into a semantic memory module
- improve data association across frames
- attach labels to output points in a richer message
- improve localization robustness for thin objects and partial occlusions
- filter background LiDAR returns when the object does not dominate the matched scan direction

The main design goal of this package is not perfect scene understanding. It is to provide a clean, debuggable bridge between Stage 1 object detection and later semantic navigation components.
