# `tb3_memory`

`tb3_memory` is Stage 3 of the TurtleBot3 perception pipeline.

Current pipeline:

```text
camera -> detector -> localizer -> semantic memory
```

Stage 1 answers:

> What does the robot see?

Stage 2 answers:

> Where is the detected object near the robot?

Stage 3 answers:

> Is this the same object I have seen before?

`tb3_memory` upgrades frame-by-frame localized observations into stable semantic objects held in a lightweight in-memory registry. It has already been implemented as an MVP and validated in practice.

## What `tb3_memory` Does

`tb3_memory` receives localized object observations from Stage 2 and maintains stable object identities over time.

In practical terms, it:

1. receives localized observations with label, confidence, and position
2. compares each observation against existing remembered objects
3. updates an existing object if it looks like the same one
4. otherwise creates a new object id
5. tracks object lifecycle over time using `times_seen`, `last_seen`, and active/stale state
6. publishes the current memory state for downstream stages

This turns short-lived per-frame observations into a more useful world-state estimate.

## What It Does Not Do Yet

The current semantic memory MVP does **not** do the following:

- no query / command resolution yet
- no navigation goal generation yet
- no long-term database persistence yet
- no scene graph or relation inference yet
- no map-frame semantic memory yet
- no advanced multi-object tracking or re-identification system

The current memory is still robot-relative and in-memory only.

## Why Semantic Memory Is Needed

Detector + localizer alone are not enough for semantic navigation.

Without memory:

- the robot would treat every frame as a new object
- repeated observations of the same person or table would not be merged
- there would be no stable identity to update over time
- later stages would not know whether an observation is new, repeated, or stale

Semantic memory provides the missing state layer between perception and reasoning:

- it keeps stable object ids over repeated observations
- it updates object state over time
- it can age or remove objects that are no longer being seen
- it creates a usable world-state topic for later query and navigation stages

## Input Topics

### `/localizer_node/localized_objects`

- Type: `vision_msgs/msg/Detection3DArray`
- Source: `tb3_localizer`

This is the main input to semantic memory.

Each incoming `Detection3D` provides:

- `results[0].hypothesis.class_id` -> detector label
- `results[0].hypothesis.score` -> confidence
- `bbox.center.position.x/y/z` -> localized position
- `header.stamp` -> observation time
- `header.frame_id` -> current frame, currently `base_link`

### Why this topic is used instead of raw `PointStamped`

The old localizer `PointStamped` output only carries position.

Semantic memory also needs:

- semantic label
- confidence
- timestamp
- frame information

So the localizer was extended to publish `Detection3DArray` on `/localizer_node/localized_objects`. This is the smallest practical interface that carries all information needed by memory while staying within standard ROS messages.

## Output Topic

### `/semantic_memory_node/objects`

- Type: `vision_msgs/msg/Detection3DArray`
- Published periodically
- Current default publish rate: `1.0 Hz`

Each published `Detection3D` represents one **active remembered object**:

- `id` -> memory object id such as `person_0`
- `results[0].hypothesis.class_id` -> detector label
- `results[0].hypothesis.score` -> smoothed / averaged confidence
- `bbox.center.position.x/y/z` -> current remembered position

Downstream stages will use this topic as the current semantic world state.

## Semantic Memory Principle

The current matching and update logic is intentionally simple and debuggable.

### Step 1: Receive localized observations

The node receives a `Detection3DArray` from the localizer. Each detection contains:

- detector label
- confidence
- localized `(x, y)` position
- timestamp

### Step 2: Group candidates by label

For each new observation, semantic memory first considers only remembered objects with the **same detector label**.

Example:

- a new `person` observation is compared only against remembered `person` objects
- it is not compared against `bench` or `stop sign`

This reduces obvious false matches and keeps the MVP logic simple.

### Step 3: Nearest-neighbor matching

Among objects with the same label, memory computes Euclidean distance:

```text
d = sqrt((x_obs - x_mem)^2 + (y_obs - y_mem)^2)
```

If the closest same-label object is within the configured distance threshold, the observation is considered the same object.

### Step 4: Update or create

- If a match is found: update the existing object
- If no match is found: create a new object id

### Why this is good enough for the MVP

This approach is simple but practical because:

- it works well when the object set is small
- it is easy to understand and debug
- it already prevents endless id creation for continuously observed objects
- it provides a stable interface for later stages without overengineering tracking

It is not a full tracking system, but it is the right level of complexity for the current pipeline.

## Object State Design

Each remembered object maintains the following fields:

- `object_id`
  - stable memory id, e.g. `person_0`, `bench_1`

- `detector_label`
  - exact YOLO label string currently used for matching

- `x`, `y`
  - current remembered position in the output frame

- `frame_id`
  - current frame of the remembered position
  - currently `base_link`

- `avg_confidence`
  - running average confidence across observations

- `times_seen`
  - how many times this object has been matched and updated

- `last_seen`
  - timestamp of the latest successful observation

- `active`
  - whether the object is still considered current / alive

In practical terms, this gives the system enough state to say:

- what object it believes exists
- where it currently believes it is
- how many times it has been seen
- how recently it was seen
- whether the memory entry is still active

## Update and Lifecycle Policy

### Update logic for matched objects

When a new observation matches an existing object:

- position is updated using exponential smoothing:

```text
x_new_mem = alpha * x_obs + (1 - alpha) * x_old_mem
y_new_mem = alpha * y_obs + (1 - alpha) * y_old_mem
```

- confidence is updated as a running average
- `times_seen` increases by 1
- `last_seen` is refreshed to the new timestamp
- `active` is set to `True`

This keeps positions stable under noisy localizer output while still allowing memory to track gradual motion or observation drift.

### Aging policy

If an object is not observed for long enough:

- after `stale_timeout` seconds, it becomes stale (`active = False`)
- after `remove_timeout` seconds, it is removed from memory entirely

Default values:

- `stale_timeout = 5.0 s`
- `remove_timeout = 30.0 s`

### Why an object may reappear as `person_1`

This is normal in the current MVP.

If an old object:

- has been removed due to timeout, or
- can no longer be matched within the distance threshold

then a later observation is treated as a new object and gets a new id such as:

- old object: `person_0`
- later reappearance: `person_1`

This does **not** necessarily mean the physical object changed. It means the current MVP could not safely preserve identity across a long disappearance or a large spatial jump.

## Validated Behavior

Current testing already showed the expected memory behavior:

- the same continuously observed object does **not** get a new id every frame
- semantic memory can keep stable ids such as `person_0`
- objects are updated over time instead of being recreated each frame
- when a target leaves view long enough, it may later reappear as a new id
- output is published stably on `/semantic_memory_node/objects`
- current publish rate is about `1 Hz`

This confirms that Stage 3 is already working as a practical MVP, not just a design concept.

## Current Supported Semantic Test Objects

The current validated semantic test set is:

| semantic_name | detector_label | gazebo_model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

These names belong to different layers:

- `semantic_name`
  - human / NLP / project-level semantic name
  - example: `table`, `stop_sign`

- `detector_label`
  - exact label produced by YOLO
  - example: `bench`, `stop sign`

- `gazebo_model`
  - Gazebo model directory name used by the simulation world
  - example: `table_marble`, `stop_sign`

## Important Naming Rule

Semantic memory matching currently uses:

- detector labels
- positions

It does **not** match directly on project semantic names.

This matters because detector labels are not always the same as semantic names.

Example:

- semantic name `table`
- detector label `bench`

So if you inspect memory output, you should expect the remembered label to be `bench`, not `table`.

## Package Structure

```text
tb3_memory/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── semantic_memory.yaml
├── launch/
│   └── semantic_memory.launch.py
└── tb3_memory/
    ├── memory_core.py
    └── semantic_memory_node.py
```

### File roles

- `tb3_memory/memory_core.py`
  - pure Python registry logic
  - matching, updating, aging, and object id creation

- `tb3_memory/semantic_memory_node.py`
  - ROS 2 node wrapper
  - subscribes to localized observations
  - publishes current memory state

- `config/semantic_memory.yaml`
  - main runtime configuration
  - matching distance, smoothing, stale/remove timeouts, publish rate

- `launch/semantic_memory.launch.py`
  - ROS 2 launch entry point

## How To Run

### 1. Launch the Gazebo test world

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py
```

### 2. Launch `tb3_detector`

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_detector detector.launch.py use_sim_time:=true
```

### 3. Launch `tb3_localizer`

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true
```

### 4. Launch `semantic_memory_node`

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tb3_memory semantic_memory.launch.py use_sim_time:=true
```

### 5. Inspect semantic memory output

```bash
source /opt/ros/humble/setup.bash
source ~/TurtleBot3-semantic-navigation/install/setup.bash
ros2 topic echo /semantic_memory_node/objects
```

Optional:

```bash
ros2 topic hz /semantic_memory_node/objects
```

## Limitations

Current limitations are intentional and acceptable for the MVP:

- matching uses only detector label + distance threshold
- nearby same-class objects may be confused
- identity is not persistent across long disappearances
- memory is still local / MVP scale
- the current system is not yet a full world model

This is enough to support the next development stage, but not enough for robust long-term semantic mapping in cluttered scenes.

## Future Work

Useful next steps include:

- map-frame support
- stronger re-identification across long disappearances
- better handling of temporary occlusion or disappearance
- support for semantic query
- support for navigation target selection
- optional persistence / save-load of memory state

The current goal of `tb3_memory` is to provide a simple, validated, debuggable semantic identity layer between localization and later reasoning or navigation components.
