# `tb3_query`

`tb3_query` is Stage 4 of the TurtleBot3 semantic navigation pipeline.

Current pipeline:

```text
camera -> detector -> localizer -> semantic memory -> semantic query
```

Stage 3 answers:

> Is this the same object I have seen before?

Stage 4 answers:

> Which remembered object does the user mean?

`tb3_query` does **not** look at raw images. It reads the current semantic memory state from Stage 3 and resolves a simple text command such as `go to the person` into one selected remembered object. It has already been implemented as an MVP and validated in practice.

## What `tb3_query` Does

`tb3_query` bridges human semantic commands and the remembered object registry.

In practical terms, it:

1. receives a text command on `/semantic_query_node/command`
2. parses it deterministically to extract a canonical semantic name
3. maps the semantic name to the corresponding YOLO detector label
4. searches the latest semantic memory snapshot for matching objects
5. selects the nearest matching object
6. publishes a `SemanticQueryResult` with the selected target

This is the handoff point between human intent and machine-actionable target selection.

## What It Does Not Do Yet

The current semantic query MVP does **not** do the following:

- no LLM parsing yet
- no advanced natural language understanding
- no attribute reasoning beyond simple phrase aliases
- no navigation execution
- no scene graph reasoning
- no direct object re-identification

The parser is intentionally deterministic and rule-based for this stage.

## Why Semantic Query Is Needed

Semantic memory stores objects, but it does not decide which object the user wants.

Without a query layer:

- there would be no way to resolve a command like `go to the table` into one specific remembered object
- downstream navigation would not know which object to approach
- there would be no bridge between human language and the detector/memory label system

Semantic query converts a human-facing command into a concrete selected target with position, identity, and confidence.

## Input Topics

### `/semantic_memory_node/objects`

- Type: `vision_msgs/msg/Detection3DArray`
- Source: Stage 3 semantic memory

This provides the current snapshot of active remembered objects. Each detection carries:

- `id` -> stable object id
- `results[0].hypothesis.class_id` -> detector label
- `results[0].hypothesis.score` -> averaged confidence
- `bbox.center.position` -> remembered position

The query node caches the latest snapshot and uses it to answer each incoming command.

### `/semantic_query_node/command`

- Type: `std_msgs/msg/String`
- Source: user / test script / teleop publisher

This carries a text command such as `go to the person` or `go to the stop sign`.

## Output Topics

### `/semantic_query_node/selected_target`

- Type: `tb3_query/msg/SemanticQueryResult`
- Published once per command

This is the key handoff into Stage 5 navigation goal generation. It carries the full resolution result including semantic name, detector label, object id, position, and confidence.

### `/semantic_query_node/query_status`

- Type: `std_msgs/msg/String`
- Published once per command

A human-readable status string such as `matched person_3 (d=2.37m, 1 candidate(s))` or `no active table in memory`. Useful for debugging and terminal monitoring.

## Naming Layers and Mapping

This project uses three distinct naming layers:

| Layer | Purpose | Example |
|-------|---------|---------|
| `semantic_name` | human / NLP / project-level name | `table`, `person`, `stop_sign` |
| `detector_label` | exact YOLO output string | `bench`, `person`, `stop sign` |
| `gazebo_model` | Gazebo model directory name | `table_marble`, `person_standing`, `stop_sign` |

Current validated mapping:

| semantic_name | detector_label | gazebo_model |
|---|---|---|
| `table` | `bench` | `table_marble` |
| `person` | `person` | `person_standing` |
| `stop_sign` | `stop sign` | `stop_sign` |

### How query uses this mapping

1. The user says `go to the table` (semantic name layer)
2. The parser extracts `table` as the canonical semantic name
3. The mapping converts `table` to detector label `bench`
4. Memory objects are searched for entries with detector label `bench`
5. The selected target carries both `semantic_name=table` and `detector_label=bench`

This mapping is loaded at startup from `semantic_targets.yaml`, the same canonical registry used by the rest of the pipeline.

## Query Principle

The current MVP query logic works as follows:

### 1. Receive and normalize command

The input text is lowercased, punctuation is removed, and whitespace is normalized.

### 2. Parse semantic name

The parser handles two cases:

- multi-word phrases via alias lookup (e.g., `stop sign` in the command text maps to canonical `stop_sign`)
- single-word targets by scanning non-filler tokens against the known target set

Filler words like `go`, `to`, `the`, `please` are skipped during parsing.

### 3. Map semantic name to detector label

Using the `semantic_targets.yaml` mapping, the canonical semantic name is converted to the corresponding detector label.

### 4. Search memory

The latest memory snapshot is filtered for objects with matching `detector_label`.

### 5. Select one target

If candidates exist, the nearest one is selected. If none exist, failure is reported.

### 6. Publish result

The `SemanticQueryResult` is published with all relevant fields populated.

## Selection Policy

The current MVP selection policy is **nearest object**.

For each matching candidate, planar distance from the robot origin is computed:

```text
r = sqrt(x^2 + y^2)
```

The candidate with the smallest `r` is selected.

Why nearest is a practical MVP policy:

- simple to explain and debug
- consistent with future navigation use
- more practical than "highest confidence" when multiple same-class objects exist at different positions
- does not depend on history beyond the current memory state

When multiple candidates exist, the log reports the number of candidates and the selected object id.

## Output Message Design

The output uses a custom message `tb3_query/msg/SemanticQueryResult`:

```text
std_msgs/Header header
bool success
string query_text
string semantic_name
string detector_label
string object_id
geometry_msgs/Point position
string frame_id
float32 confidence
string status_message
```

### Why a custom message was used

A standard message like `Detection3DArray` does not carry `semantic_name` or `query_text`, and Stage 4 is the point where human semantic intent must be explicitly reconciled with detector-level labels. A small custom message avoids fragile encoding conventions and makes the downstream Stage 5 integration cleaner.

### Field purposes

- `success` — cleanly separates failure from valid selection
- `query_text` — original command for logging and debugging
- `semantic_name` — the human-facing name extracted from the command
- `detector_label` — the YOLO label used to search memory
- `object_id` — stable memory identity such as `person_3`
- `position` — the remembered (x, y) of the selected object
- `frame_id` — coordinate frame of the position (currently `base_link`)
- `confidence` — smoothed confidence from memory
- `status_message` — human-readable result or error

## Validated Behavior

Current testing showed the expected query behavior:

- `go to the person` resolves successfully to the nearest active person object
- `go to the table` resolves successfully when bench is in memory
- `go to the stop sign` resolves to `stop_sign` via alias mapping
- unsupported targets like `go to the chair` fail with a clear message
- selected target contains correct `object_id`, `semantic_name`, `detector_label`, `position`
- smoke test passed 5/5 cases in end-to-end Gazebo simulation

## Package Structure

```text
tb3_query/
├── package.xml
├── CMakeLists.txt
├── msg/
│   └── SemanticQueryResult.msg
├── config/
│   └── semantic_query.yaml
├── launch/
│   └── semantic_query.launch.py
└── tb3_query/
    ├── query_core.py
    └── semantic_query_node.py
```

### File roles

- `tb3_query/query_core.py`
  - pure Python logic: command parsing, mapping loader, target selection
  - no ROS dependency

- `tb3_query/semantic_query_node.py`
  - ROS 2 node wrapper
  - subscribes to memory and command topics
  - publishes SemanticQueryResult and debug status

- `msg/SemanticQueryResult.msg`
  - custom message definition for selected target output

- `config/semantic_query.yaml`
  - runtime configuration for topics, mapping file path, and output frame

- `launch/semantic_query.launch.py`
  - ROS 2 launch entry point

### Build note

This package uses `ament_cmake` with `rosidl_default_generators` to support custom message generation alongside Python implementation.

## How To Run

### 1. Launch the full perception pipeline

```bash
cd ~/TurtleBot3-semantic-navigation
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

# Terminal 1: Gazebo
ros2 launch tb3_frontier_exploration detector_test_sim.launch.py

# Terminal 2: detector
ros2 launch tb3_detector detector.launch.py use_sim_time:=true

# Terminal 3: localizer
ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true

# Terminal 4: memory
ros2 launch tb3_memory semantic_memory.launch.py use_sim_time:=true

# Terminal 5: query
ros2 launch tb3_query semantic_query.launch.py use_sim_time:=true
```

### 2. Send commands

```bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':')
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 topic pub --once /semantic_query_node/command std_msgs/String "data: 'go to the person'"
ros2 topic pub --once /semantic_query_node/command std_msgs/String "data: 'go to the table'"
ros2 topic pub --once /semantic_query_node/command std_msgs/String "data: 'go to the stop sign'"
```

### 3. Inspect results

```bash
ros2 topic echo /semantic_query_node/selected_target
ros2 topic echo /semantic_query_node/query_status
```

### 4. Run smoke test

```bash
cd ~/TurtleBot3-semantic-navigation
python3 src/tb3_query/test/smoke_test_query.py
```

## Limitations

Current limitations are intentional and acceptable for the MVP:

- simple deterministic parsing only
- no rich attribute reasoning yet
- dependent on current semantic memory state
- if no active object exists in memory, query fails
- if multiple same-class objects exist, the nearest one is always selected
- phrase alias mapping is hardcoded for the current target set

## Future Work

Useful next steps include:

- richer command parsing with synonym support
- attribute filtering such as "the table near the wall"
- support for map-aware selection when world-frame positions are available
- stronger selection logic when multiple same-class objects exist
- integration with LLM-based intent parsing for open-vocabulary commands
- tighter integration with Stage 5 nav goal adapter

The current goal of `tb3_query` is to provide a clean, validated, debuggable bridge between human semantic commands and the downstream navigation pipeline.
