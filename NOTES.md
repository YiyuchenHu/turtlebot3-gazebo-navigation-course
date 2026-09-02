# NOTES — design rationale, measurements, and hard-won lessons

This file is the "why" of the course workspace. It is written for instructors
who have to defend the design choices, for students who want to know *why* a
parameter has the value it has, and for whoever maintains the course next and
is tempted to change something that looks arbitrary.

It is deliberately long and discursive. Most of it is background rather than
instruction — but **§7 is the operational reference**: the stub baseline
INSTRUCTIONS Step 2 asks you to record, the debugging procedure, and the
interface notes all live there, and INSTRUCTIONS points here for them.

- **Setup and operation** → [README.md](README.md)
- **The assignment itself** → [INSTRUCTIONS.md](INSTRUCTIONS.md)
- **Why any of it is the way it is** → this file

Every measurement reported below — landmark errors, timings, confidences,
beam-hit fractions, inference cost — was taken on this workspace with the
shipped `yolo26n.pt` weights, in Gazebo Classic 11 under ROS 2 Humble. Numbers
that are *not* measurements are labelled where they appear: parameter values and
config defaults, values derived arithmetically or geometrically from a
measurement, and approximations, which are written with a leading `~`.

## Contents

- [0. What the assignment teaches](#0-what-the-assignment-teaches)
- [1. Why these three targets, and what was retired](#1-why-these-three-targets-and-what-was-retired)
- [2. The perception parameters are one interlocked set](#2-the-perception-parameters-are-one-interlocked-set)
- [3. Why `conf_threshold` is 0.35](#3-why-conf_threshold-is-035)
- [4. `class_filter` and the airplane problem](#4-class_filter-and-the-airplane-problem)
- [5. Known simplifications (intentional, documented)](#5-known-simplifications-intentional-documented)
- [6. Measured results, and how they were measured](#6-measured-results-and-how-they-were-measured)
- [7. Advanced debugging](#7-advanced-debugging)
- [8. Notes for whoever maintains this next](#8-notes-for-whoever-maintains-this-next)
- [9. Bonus / extension material](#9-bonus--extension-material)

---

## 0. What the assignment teaches

The detector is one file, but the reason it is *this* file is the shape of what
surrounds it:

1. **Integrating a pretrained deep-learning model into a live ROS 2 node** —
   Python dependencies, model weights, and inference that has to keep up with a
   camera stream.
2. **Programming against a message contract.** The detector's output feeds a
   chain of nodes the student did not write. Wrong boxes, wrong labels or a
   wrong coordinate convention do not raise an exception; they surface metres
   away, as the robot driving to the wrong place. §2 is that lesson in full.
3. **Understanding a complete semantic-navigation pipeline** — detection →
   camera/LiDAR fusion → semantic memory → language-style commands → Nav2
   goals, all running alongside SLAM and autonomous exploration.

---

## 1. Why these three targets, and what was retired

The course ships three semantic targets — **person**, **trash can**, **chair** —
and that set is not arbitrary. It is what survived measurement. Two earlier
targets were retired on 2026-08-27, and several candidate replacements were
rejected before `chair` was chosen. The failures are more instructive than the
successes, so they are documented here in full.

### 1.1 The two retirements

Both failures were in the **LiDAR**, not in the detector — which is exactly why
they were confusing at first. The camera happily saw something; the range
window did not.

| retired target | measured reason |
|---|---|
| `table` (`table_marble`) | Its `<link>` is posed at `z=0.648`, so the geometry sits **above** the 0.121 m LiDAR scan plane. 6 of 9 test poses returned no range at all (beam-hit fraction 0.15). The detector was no better: yolo26n's best `bench` confidence was only p50 **0.21**. Failed detection *and* failed ranging. |
| `stop_sign` | Ranging was accurate when it hit — error 0.01–0.02 m — but the pole is so thin that only **16%** of the beams in the ±5° window landed on it (beam-hit fraction 0.16). The windowed median therefore came from the wall behind, which put the landmark **4.38 m** from the real sign. Its own `stop sign` class only reached p50 0.25; on most poses the network called it `airplane` (p50 0.60–0.64). |

The `stop_sign` case is the sharper of the two. Its per-beam accuracy was
excellent — better than any other target. It still produced a landmark over
four metres from the object, because the statistic that mattered was not the
accuracy of a hit but the *fraction* of the window that hit anything. An
84%-wall / 16%-sign window has a wall for a median. See §2 for why the
median is used at all.

Note also that `table_marble` was the reason `conf_threshold` used to be 0.12
(§3) and the reason `scan_window_half` was once 10 (§2). Retiring one bad
target let two unrelated parameters return to sane values. Bad targets
propagate.

### 1.2 Why the replacement candidates were rejected

When a third target was needed, the shortlist was measured the same way —
detector confidence over a set of test poses, and beam-hit fraction in the
range window — before anything was wired into `semantic_targets.yaml`:

| candidate | outcome |
|---|---|
| Sofa, MiniSofa | COCO `couch` **never fired**. Not a low score — no detections at all. |
| Fridge | COCO `refrigerator` **never fired**. |
| Suitcase1 | `suitcase` reached only p50 **0.14**, far below any usable threshold. |
| Fuel `Chair` | Detected *better* than the eventual winner (**0.65** vs 0.52), but its thin legs gave a beam-hit fraction as low as **0.09**, with 2 of 9 poses returning nothing. The stop-sign failure mode exactly. |
| **VisitorChair** ✅ | Lower confidence (0.52) but a solid, continuous body in the scan plane. **Chosen.** |

That last row is the interesting one: the better-detected model lost. Picking
targets by detector score alone would have shipped a second `stop_sign`.

### 1.3 The generalisable rule

An object is usable as a landmark in this stack only if it is **both**

1. recognisable to the camera (a COCO class that actually fires, at a
   confidence comfortably above threshold), **and**
2. solid to the LiDAR between the floor and roughly 0.3 m — wide enough and
   continuous enough that a majority of the beams in the range window land on
   it rather than past it.

Either condition alone is worthless. `table_marble` failed both. `stop_sign`
passed (1) badly and (2) catastrophically. Fuel's `Chair` passed (1) best of
all and failed (2). Only the intersection ships.

This is a genuinely transferable robotics lesson: perception targets are
chosen against the *whole sensor suite*, not against the model zoo.

### 1.4 Why `trash can` is detected as `traffic light`

COCO has no trash-can class. yolo26n reports the trash-can model as
`traffic light`, and `semantic_targets.yaml` maps `traffic light` → `trash_can`:

| `semantic_name` (task level) | `detector_label` (COCO) |
|---|---|
| `person` | `person` |
| `trash_can` | `traffic light` |
| `chair` | `chair` |

That mismatch is the whole reason `semantic_name` and `detector_label` are
separate fields in the first place. A detector must report what the network
actually says; the *meaning* of a label is a downstream, task-level decision.
Any `infer()` that "helpfully" renames `traffic light` to `trash_can` breaks
the mapping — and breaks it silently, because the raw label simply stops
matching the whitelist and the object quietly disappears.

One visible consequence: landmark IDs are built from the **raw detector
label**, so the live IDs read `person_0`, `traffic light_0` (with a space
inside it) and `chair_0` — not `trash_can_0`. That space bites people writing
scripts that parse landmark IDs.

### 1.5 The self-test world still holds the retired props

`src/tb3_bringup/worlds/detector_test.world` — the static world
behind `detector_test.launch.py`, used in INSTRUCTIONS Step 4 — places
`person_standing`, `table_marble` and `stop_sign` in front of the camera. It
predates the 2026-08-27 target change and was deliberately left alone: it is an
*isolated detector* test, and the two retired props were retired for LiDAR
reasons (§1.1) that a camera-only test never exercises.

The consequence to expect: three objects are visible in Gazebo, and with the
shipped `class_filter` only `person` is reported. That is enough to prove
`infer()` works, and it is not a bug in the student's code.

---

## 2. The perception parameters are one interlocked set

**This is the most important section in this file.**

Every parameter in the perception chain — detector confidence, class whitelist,
LiDAR window width, memory association distance — looks like a local knob. None
of them is. They are one interlocked set, and the fastest way to break this
stack is to tune one layer to compensate for a problem in another.

Here is a real example from this repository's history.

### 2.1 First, how a box becomes a coordinate

You need the mechanism to follow the story.

`tb3_localizer` takes the bounding box **centre x**, maps it linearly onto the
camera's **62.2° horizontal FOV** to get a bearing, then reads a window of
LiDAR ranges centred on that bearing and takes the **median** of the window.
Bearing plus range is projected onto the ground plane to give `(x, y)` in
`base_link`.

Two consequences fall straight out of that:

- A horizontally shifted box is a wrong bearing, and a wrong bearing localises
  the object in the wrong *direction* — a detector-side error that shows up as
  a navigation-side symptom.
- The width of the range window is set by `scan_window_half` (in degrees on
  either side of the bearing). The median is used because it is robust: a few
  stray beams past the edges of an object do not move it. That robustness has a
  hard limit — a median only survives if a *majority* of the window is on the
  object. Past 50% off-target, the median stops being a robust estimate of the
  object and becomes an accurate estimate of whatever is behind it.

### 2.2 The `scan_window_half` 5 → 10 → 5 story

`tb3_localizer`'s `scan_window_half` was once raised from **5 to 10**. The
reason was perfectly sensible in isolation: the old `bench` target
(`table_marble`, see §1.1) subtended a very wide bounding box — about **32°** —
and a narrow ±5° LiDAR window kept missing it. Widen the window, catch the
bench. Locally correct, one-line change, obvious win.

But `scan_window_half` is shared by **every** class.

At ±10°, the window spans **0.67 m at 1.9 m range** — which is *wider than a
person*. Most of the rays in the window flew straight past her and hit the wall
**0.9 m behind**. Majority off-target, so the windowed median came back as the
wall. The projected position landed behind the person. And the `person`
landmark **stopped forming altogether**.

Read that chain again, because the shape of it is the lesson: a change made to
accommodate a **detector-side** property of one target (a wide bbox on the
bench) silently destroyed a **different** target, **two stages downstream**, in
a node nobody had touched.

And the symptom pointed nowhere near the cause. What a student or instructor
actually observes is:

> "the person never appears in RViz any more"

Nothing in that sentence mentions the bench, the LiDAR, or a window width. The
detector is fine. The person is detected every frame, with p50 ~0.9 confidence.
The debug image is full of correct green boxes. And the landmark never forms.

When the bench was retired (§1.1), `scan_window_half` went back to **5**, and
the person landmark came back.

### 2.3 The moral

Three things worth stating explicitly:

1. **A shared parameter is a coupling.** `scan_window_half` reads like a
   localizer detail. It is really a contract between the *geometry of every
   target* and the *statistics of the range estimator*. Widening it to suit
   the widest object narrows the margin for the narrowest one.
2. **Compensating tuning hides faults.** The window was widened to compensate
   for a target that should never have shipped. Once you tune layer B to paper
   over layer A, the real fault is no longer visible anywhere — it has been
   redistributed into two parameters that are each individually defensible.
3. **The symptom's location tells you nothing about the fault's location.**
   In a pipeline, a failure surfaces at the first stage that *cannot proceed*,
   which is generally well downstream of the stage that is wrong.

### 2.4 So: walk the chain before you touch a parameter

When a target misbehaves, do not start by changing `conf_threshold`. Find out
which stage actually drops the object, by walking the chain in order with
`ros2 topic echo`:

```text
/detector_node/detections            (tb3_detector)
        ↓
/localizer_node/localized_objects    (tb3_localizer)
        ↓
/semantic_memory_node/objects        (tb3_memory)
        ↓
/semantic_map_memory_node/landmark_objects   (semantic map memory)
```

The **first silent topic** tells you which stage lost the object, and therefore
which parameter is even eligible to be the cause. In the `scan_window_half`
story, `/detector_node/detections` was healthy and
`/localizer_node/localized_objects` was empty — which localises the fault to
the localizer in about thirty seconds, and rules the detector out entirely.

Only after the chain has named a stage should any threshold be touched.

---

## 3. Why `conf_threshold` is 0.35

The shipped value in `config/detector.yaml` is **0.35**, with
`class_filter: ["person", "traffic light", "chair"]`.

**It costs nothing.** All three real targets score far above it:

| target | measured confidence |
|---|---|
| person | p50 ~0.90 |
| chair | p50 0.87 |
| trash can | 0.31–0.65 |

**What it buys is protection from ghosts.** Dropping the threshold to **0.30**
was enough for yolo26n to call something **0.36 m from the robot** a
`traffic light` and plant a phantom trash-can landmark right next to the
person. That is not a cosmetic problem: `go to trash can` then drives to a
piece of hallucinated furniture, and — because the phantom is next to the
person — the failure looks like a *navigation* bug rather than a detector one.

The threshold used to be **0.12**. That value existed for exactly one reason:
to scrape `table_marble` in as a `bench` (§1.1). With that target retired, the
threshold no longer has to be low, and it is not.

The trade-off in both directions:

- **Too high** → the chair, weakest of the three head-on, never appears.
- **Too low** → ghost landmarks, as above.
- Start from the shipped 0.35.

You may tune `conf_threshold` in `config/detector.yaml` while developing, but
the acceptance test runs with the shipped config.

One specific misuse to name and rule out: the Nav2 rotate/abort wedge described
in the README troubleshooting section is **not** a detector problem and is not
fixable from the detector. Loosening `conf_threshold` or widening
`class_filter` to chase a navigation symptom is precisely how you end up with
the ghost landmarks documented above — a real fault swapped for a worse one.

---

## 4. `class_filter` and the airplane problem

`class_filter` is a **whitelist**, and that is a load-bearing design decision
rather than a stylistic one.

yolo26n reports COCO **`airplane`** on most untextured Gazebo props:

- p50 **0.51** on a cafe table
- p50 **0.34** on a *person*
- **100% of frames** on some objects

The obvious hypothesis is that this is background noise — a model producing
junk on out-of-distribution imagery. It is not, and there is a control result
that settles it: **an empty world detects nothing at all.** No props, no
detections. So `airplane` is not a floor of random activations; the Gazebo
models genuinely look like that to the network.

That distinction matters, because it determines what you can do about it. Noise
could be thresholded away. This cannot be, usefully: 0.51 on a cafe table sits
*inside* the trash can's own measured confidence range (0.31–0.65). A threshold
high enough to exclude that `airplane` would throw away most real trash-can
detections along with it, so no setting of one number separates them.

And even at high confidence, `airplane` is useless as a landmark class: it
fires on **several different objects at once**, so it can never identify any
one of them. A landmark built from a class that means "some untextured prop" is
not a landmark, it is a coordinate with a misleading name attached.

The whitelist is the only thing keeping it out. Therefore:

- **Never add `"airplane"` to `class_filter`.**
- **Never replace the whitelist with `[""]`** ("accept everything") to "see
  more". `[""]` means *all classes*, which means `airplane` on every prop in
  the room, which means poisoned landmarks.
- There is a separate, purely mechanical trap here: a bare `[]` in YAML
  triggers an rclpy Humble type-inference crash, so `[""]` exists as the
  "all classes" spelling. Its existence as valid syntax is **not** an
  invitation to use it for this course's config.

---

## 5. Known simplifications (intentional, documented)

These are deliberate. They are documented rather than hidden because each one
is a reasonable discussion topic, and because an undocumented simplification is
indistinguishable from a bug when someone hits it.

### 5.1 `tb3_nav_adapter` frame assumption

`compute_approach_pose` treats the target coordinates as if they were
robot-relative (robot at the origin), but the configured pipeline feeds it
**map-frame** landmarks. The 0.5 m standoff is therefore computed along the
*map-origin → target* direction rather than *robot → target*.

In these small rooms the map origin equals the spawn pose, so the error is
modest and Nav2 still reaches the target — but it is a real simplification
worth understanding. The proper fix is a TF lookup of `base_link` in `map` at
query time, so the standoff is computed along the actual approach direction.
See §9 for this as an exercise.

There is one case where the simplification stops being cosmetic and becomes a
hard stop, and it is worth tracing through the code because the symptom is a
silence rather than an error. `compute_approach_pose` computes
`dist = hypot(tx, ty)` and returns `None` when that is below
`min_standoff_distance` (0.3 in `nav_goal_adapter.yaml`). Because `tx, ty` are
map coordinates, `dist` is the distance from the **map origin**, so a landmark
standing near the origin is rejected as "too close" no matter where the robot
is. `nav_goal_adapter_node` logs `Target ... too close, skipping goal` and
publishes nothing; the coordinator arms its `nav_goal_timeout_sec` only once a
goal pose arrives, so it waits in `SEMANTIC_NAV` indefinitely and no
`[TARGET_REACHED]` or `[TARGET_FAILED]` is ever published.

The default world has exactly such a figure: `person_centre` stands at Gazebo
`(0, 0)` (§6.2), and the map frame coincides with the world frame. Between
`min_standoff_distance` 0.3 and `approach_distance` 0.5, a landmark inside
0.8 m of the map origin cannot yield a usable goal.
`scripts/acceptance_run.sh` checks that radius before issuing a command and
reports `SKIPPED` with the reason instead of stalling for its full timeout; it
also defaults to `--world warehouse_models`, which has no such target.

### 5.2 The inert `odom_topic`

`tb3_frontier_exploration/config/params.yaml` names an
`odom_topic: /odometry/filtered` that **does not exist** in this stack. The
exploration nodes actually obtain the robot pose via TF, so the setting is
inert. Don't let it mislead you: changing it does nothing, and its presence is
not evidence that an EKF is running somewhere.

### 5.3 Landmark IDs are memory slots, not identities

Landmark IDs (`person_0`, `person_1`, …) are assigned in **observation order**
by semantic memory. They are memory slots, not person identities: the same
physical figure can receive a different index across runs, and there is no
re-identification anywhere in the stack.

This is why the acceptance criteria ask for "at least two different *observed*
`N`" rather than naming specific indices, and why `go to person` (nearest)
behaves predictably while `go to person 3` depends on exploration order. Use
`ros2 topic echo /semantic_map_memory_node/landmark_objects` to see the live
mapping from ID to map coordinates in any given run.

Combined with §1.4, the IDs you will actually see in the two shipped worlds are
`person_0 … person_4` (default world), and `person_0`, `traffic light_0`,
`chair_0` (the `warehouse_models` world) — with a space inside
`traffic light_0`.

---

## 6. Measured results, and how they were measured

### 6.1 Full-stack acceptance run (`world:=warehouse_models`)

Landmark position error, against Gazebo ground truth:

| landmark | error vs Gazebo truth |
|---|---|
| person | **0.11 m** |
| trash can | **0.02 m** |
| chair | **0.20 m** |

Navigation outcomes, one command at a time:

| command | result | time | final robot distance to the real object |
|---|---|---|---|
| `go to person 0` | SUCCEEDED | 14.4 s | 0.78 m |
| `go to trash can` | SUCCEEDED | 7.2 s | 0.62 m |
| `go to chair` | SUCCEEDED | 9.3 s | 0.92 m |

The ordering in that second table is informative: the chair has both the
largest landmark error (0.20 m) and the largest final distance (0.92 m).
Landmark error propagates almost directly into approach error, which is the
whole reason the criterion is stated in metres from the *real object* rather
than as a Nav2 status.

### 6.1.1 Where the 1.2 m acceptance bar comes from

The criterion was **1.0 m** and is now **1.2 m**. That is not slack added to
make runs pass; it is the error budget of a correctly working stack, which
1.0 m did not actually cover.

Three terms stack up, and all three are by design:

| term | size | why it is there |
|---|---|---|
| nav-adapter standoff | **0.50 m** | The approach pose is deliberately offset from the landmark so the robot stops *in front of* the object instead of driving into it (§5.1). The robot is supposed to end ~0.5 m away. |
| landmark bias toward the robot | **0.15–0.36 m** (chair, measured) | The localizer fuses the bbox bearing with the LiDAR range, and LiDAR returns the **near surface**, not the object centre. The landmark therefore sits systematically short of the true centre, along the line of sight. Bulky, irregular objects bias worst: the chair measured 0.15–0.36 m across six runs, against 0.02–0.15 m for the trash can and 0.06–0.12 m for a person. |
| Nav2 xy goal tolerance | **~0.25 m** | `general_goal_checker.xy_goal_tolerance` in the shipped Nav2 params. Nav2 declares the goal reached anywhere inside that radius, and the error can point away from the object. |

Worst case those compose to roughly **1.1 m** with nothing wrong anywhere,
which left 1.0 m with negative margin rather than tight margin. Six repeat
runs on `world:=warehouse_models` bore that out: chair finished between 0.77
and 1.04 m, and one otherwise-clean run — all three commands `TARGET_REACHED`,
every landmark within tolerance — was scored FAIL purely because the chair
ended at 1.04 m. That is the bar mis-measuring, not the stack failing.

1.2 m keeps roughly 0.1 m over the worst legitimate case while staying far
below the failure modes the criterion exists to catch. Those are not near
misses: a mislocalised landmark of the `stop_sign` variety (§1.1) put the
robot **4 m** from the real object. Nothing in the gap between 1.2 m and 4 m
has ever been observed, so widening by 0.2 m costs no diagnostic power.

### 6.2 How ground truth was obtained

Gazebo will tell you where a model actually is:

```bash
gz model -m <entity> -p
```

The first line of stdout is `x y z roll pitch yaw`, space-separated. The robot
reads the same way, with entity name `waffle_pi`.

The **map frame was measured to coincide with the Gazebo world frame to within
0.015 m** in this stack, which is a full order of magnitude below the landmark
errors in §6.1. Map-frame landmark coordinates may therefore be compared
directly against Gazebo world coordinates without a transform — the comparison
error is negligible against what is being measured.

Entity names, for anyone scripting a check:

**`warehouse_semantic_models.world`** (`world:=warehouse_models`), robot spawn
`(-1.2, -1.2)`:

| entity | world position |
|---|---|
| `semantic_person` | `(0.60, -2.20)` |
| `semantic_trash_can` | `(0.70, 1.50)` |
| `semantic_chair` | `(-1.30, 2.40)` |

**`warehouse_models_person.world`** (`world:=warehouse_models_person`,
default), robot spawn `(-1.5, 0.0)`:

| entity | world position |
|---|---|
| `person_corner_ne` | `(2.0, 2.0)` |
| `person_corner_nw` | `(-2.0, 2.0)` |
| `person_corner_sw` | `(-2.0, -2.0)` |
| `person_corner_se` | `(2.0, -2.0)` |
| `person_centre` | `(0.0, 0.0)` |

Note that the default world contains **five persons and no trash can and no
chair**, which is why the trash-can and chair criteria name
`world:=warehouse_models` explicitly.

### 6.3 Why `TARGET_REACHED` alone is not the acceptance criterion

Nav2 will happily report success at a **mislocalised** landmark. Its goal is
the landmark; if the landmark is 4 m from the real object (as `stop_sign`
managed in §1.1), Nav2 drives to the wrong place, arrives, and reports
`TARGET_REACHED` with complete confidence. The status message is a statement
about the *navigation stack*, not about the world.

That is the entire reason acceptance is defined as "the robot ends within
1.2 m of the **real** object", measured against Gazebo ground truth via §6.2,
and why any automated acceptance check has to query Gazebo rather than trust
the status topic.

### 6.4 Detector cost

| model | inference time |
|---|---|
| `yolo26n` at `imgsz` 640, CPU | **36.9 ms/frame** |
| `yolov8n` at `imgsz` 640, CPU (for comparison) | 36.1 ms/frame |

The two are within a millisecond of each other on CPU, so the newer weights
cost effectively nothing. This is why CPU inference is sufficient for the whole
course and a GPU is optional.

Pinned dependency: **ultralytics 8.4.31** (yolo26n needs >= 8.4.x, and the
course is validated on that exact version). Weights: `yolo26n.pt`, **5.54 MB**,
git-ignored — downloaded, never committed.

---

## 7. Advanced debugging

### 7.1 Baseline: what the stub does before anything is implemented

On `main`, `tb3_detector` ships as a stub. Knowing its exact behaviour is
useful, because "the stub is working correctly" and "my implementation is
broken" look identical from a distance:

- The robot performs a warm-up rotation, then **explores and maps the room
  autonomously** — frontier exploration, Nav2 and SLAM are all live and do not
  depend on the detector at all.
- RViz (started by `nav.launch.py`, T2) shows the growing occupancy map, the
  frontier markers exploration is chasing, the Nav2 costmaps, and the
  **Detector Debug Image** panel. All of those are alive with the stub in
  place; only the boxes are missing.
- `ros2 topic echo /detector_node/detections` shows empty `detections: []`
  arrays streaming **at camera rate**. Empty is not the same as silent: the
  topic is alive.
- The RViz **Detector Debug Image** panel shows the raw camera stream with no
  boxes, because the stub forwards the image unannotated.
- Object commands are accepted but **fail gracefully**: semantic memory stays
  empty, so `go to person 0` answers
  `query failed: no active person in memory` on `/coordinator_node/status`,
  and exploration resumes automatically ~3 s later.

That last message is the useful "before" snapshot: the whole stack is idling,
waiting for detections that never come.

### 7.2 Look at the debug image first

`rqt_image_view /detector_node/debug_image`.

- **No boxes there** → detection problem. Stop; the rest of the chain is
  irrelevant until boxes appear.
- **Boxes there but no landmarks in RViz** → the detector is fine. Check the
  chain *after* the detector, in order (§7.3). This is exactly the situation
  the `scan_window_half` story in §2.2 produced.

### 7.3 Walk the pipeline with `ros2 topic echo`, in order

```text
/detector_node/detections
    → /localizer_node/localized_objects
        → /semantic_memory_node/objects
            → /semantic_map_memory_node/landmark_objects
```

The **first silent topic** tells you which stage lost your object. This single
habit resolves most "it doesn't work" reports without anyone editing a
parameter, and it is the discipline §2 is arguing for.

For the map-level topic, useful message fields are
`results[0].hypothesis.class_id` (the raw detector label) and
`bbox.center.position.x` / `.y` (map frame).

### 7.4 Runtime statistics overlay

Start T3 with the debug node enabled:

```bash
ros2 launch tb3_bringup backend.launch.py use_runtime_debug:=true
```

This adds `semantic_runtime_debug_node`, which logs per-stage counts and
per-class confusion diagnostics as CSV under `/tmp/semantic_debug`. It is the
right tool when the fault is statistical rather than binary — "the chair
appears sometimes" is a question about rates, and eyeballing a topic echo will
not answer it.

### 7.5 Coordinator status strings

`/coordinator_node/status` (`std_msgs/String`) is formatted exactly as
`[<MODE>] <text>`, with modes `EXPLORING`, `SEMANTIC_QUERYING`, `SEMANTIC_NAV`,
`TARGET_REACHED`, `TARGET_FAILED`. Useful lines to recognise:

```text
[SEMANTIC_NAV] target selected: <id> (<semantic_name>) at (<x>, <y>) — waiting for goal pose
[SEMANTIC_NAV] goal pose received: (<x>, <y>) in map — sending to Nav2
[TARGET_REACHED] Nav2 goal succeeded — target reached
[TARGET_FAILED] query failed: <reason>
```

The terminal states are a status beginning `[TARGET_REACHED]` (success) or
`[TARGET_FAILED]` (failure). A parse or lookup failure also surfaces as
`[TARGET_FAILED]`, **not** as `[SEMANTIC_QUERYING]`: `coordinator_node.py`
calls `_set_mode(Mode.TARGET_FAILED)` on the line *before* it publishes
`query failed: ...`, and the prefix is stamped from the mode at publish time.
Match on the substring `query failed:` if you want that case specifically.

**If you match these strings programmatically**, note the em dashes (U+2014)
inside them. Match on ASCII-safe substrings — `target selected:`, the
`[TARGET_REACHED]` prefix — never on a whole line containing a dash.

### 7.6 Environment and interface notes

- **`use_sim_time`**: every course launch (T1–T5 and the one-command shell)
  already defaults to `use_sim_time:=true`, so no flag is needed in
  simulation. Pass `use_sim_time:=false` only if you reuse a node on a real
  robot.
- **The `.msg` files are frozen.** `SemanticQueryResult` in
  `src/tb3_query/msg/` is compiled into **three** packages — `tb3_query`,
  `tb3_nav_adapter`, `tb3_coordinator`. Changing it forces a full rebuild of
  all three and breaks the grader's interface contract, which is why the
  assignment forbids editing it outright rather than merely discouraging it.
- **The camera publishes BEST_EFFORT.** Gazebo's camera plugin, like every ROS
  sensor stream, uses BEST_EFFORT reliability. If you write your own
  subscription to `/camera/image_raw` (or `/camera/camera_info`) with the rclpy
  default, which is RELIABLE, the QoS profiles are incompatible: the
  subscription never matches the publisher, no callback ever fires, and
  **nothing is logged**. `ros2 topic info -v /camera/image_raw` shows the
  mismatch. `/detector_node/detections` and `/detector_node/debug_image` are
  RELIABLE in the other direction — see the QoS column in INSTRUCTIONS. This
  matters most for the §9.1 bonus, which subscribes to the camera directly.
- **`class_filter` entries are COCO detector labels**, and multi-word COCO
  labels contain a space (`"traffic light"`, not `"trash_can"`). See §1.4 for
  where that space resurfaces in landmark IDs.

### 7.7 Why the launches are split six ways

Each of T1–T5 is standalone: starting one without its upstream neighbours never
crashes, because every node subscribes and waits rather than requiring its
input to exist at start-up. That is what makes it safe to `Ctrl-C` T5 and
re-run it on every edit while Gazebo, SLAM and Nav2 keep their state — the
development loop INSTRUCTIONS Step 3 describes.

`full_stack.launch.py` includes the same five sub-launches with fixed
start-up delays standing in for the "wait until ready" checks a human performs
by hand. It is right for a demo or a smoke test and wrong for development:
everything shares one log stream, and restarting the detector alone is no
longer possible. The delays are also fixed, so on a slow machine the ordering
guarantee the six-terminal flow gives you is not guaranteed at all.

### 7.8 The failed-goal set, and the 40 s limit cycle it replaced

`goal_assignment_node` remembers goals that failed, so it does not immediately
re-send one Nav2 has just given up on. That memory used to be a **single slot**
(`last_failed_goal_`), and that single slot was a deadlock:

```text
frontier goal (0.00, -1.36)  stalls 20 s -> CANCELED -> last_failed = (0.00, -1.36)
fallback goal (-1.20, 0.30)  stalls 20 s -> CANCELED -> last_failed = (-1.20, 0.30)   <-- overwrites
frontier goal (0.00, -1.36)  is no longer "the last failed goal" -> re-armed
... repeat forever, period = 2 x goal_timeout = 40 s
```

The robot never physically jams — the nearest obstacle stays outside the
inflation radius throughout. It just alternates between two goals it cannot
reach while the map stops growing. The signature is unmistakable in the logs:
in a wedged run the most-frequently-selected goal accounts for **100 %** of all
selections; in a healthy run it is 6–20 %.

The fix has two halves, and neither works alone:

* **A time-limited failure set.** Frontier and fallback goals now share one set
  and cannot overwrite each other. Repeated failures at the same spot refresh
  the timestamp rather than appending, so one bad location cannot flood the
  set, and `failed_goal_max_entries` (64) bounds it. Entries expire after
  `failed_goal_ttl_sec` (90 s) — TTL, not success, is what re-admits a goal.
  Success deliberately does **not** clear the memory: if it did, any fallback
  goal that happened to succeed would re-arm a frontier that had been failing
  for minutes, which is the original bug wearing a different hat.
* **A minimum-uptime guard.** The stuck-detector used to require "the robot has
  moved at least once", which is exactly false during this deadlock, so the
  escape hatch was shut when it was needed. It is now gated on
  `min_exploration_time_sec` (60 s) measured from the moment exploration is
  genuinely running (enabled + warmup done + Nav2 answering), not from node
  construction — `startup_warmup_timeout_sec` can be 90 s, which would
  otherwise eat the whole guard window.

Measured over 12 probe runs after the fix: 0 wedged, and the most-frequent-goal
share dropped to a maximum of 12 %, squarely inside the healthy band.

---

## 8. Notes for whoever maintains this next

Two operational hazards that cost real debugging time and are not obvious from
reading the code.

**The clean-restart `pkill` patterns can kill the process that runs them.**
The README's clean-restart procedure uses `pkill -9 -f '<pattern>'` with long
alternation patterns (`detector_node|localizer_node|coordinator_node|…`).
`pkill -f` matches against the full command line of every process — *including
the command line of whatever is invoking it*. If a script, a `bash -c` string,
or a tmux command carries those node names in its own argv, it matches itself
and dies mid-cleanup. **This actually happened during development.** Guard
against it: put the cleanup in a file and execute the file rather than passing
an inline `-c` string, use patterns that cannot appear in the caller's argv, or
kill by recorded PID.

**A conda base environment breaks the build.** The README says to run
`conda deactivate` until `(base)` leaves the prompt, without saying why: a conda
Python shadowing `/usr/bin/python3` on `PATH` breaks colcon and, in particular,
`rosidl` message generation, in ways whose error messages point at the message
package rather than at the interpreter. It costs an afternoon the first time.

**`Managed nodes are active` is ambiguous.** When waiting for Nav2 to come up
in T2, the readiness signal is
`[lifecycle_manager_navigation]: Managed nodes are active`.
`lifecycle_manager_slam` prints the **same** `Managed nodes are active` text
and fires **earlier**. Any automated wait must require the `_navigation`
substring on the same line, or it will proceed while Nav2 is still starting and
produce a confusing cascade of downstream failures.

---

## 9. Bonus / extension material

### 9.1 Rewrite the localizer

For extra credit, re-implement `tb3_localizer`'s node (`localizer_node.py` /
`localizer_core.py`) from scratch against its interface: consume
`/detector_node/detections` + `/scan` + `/camera/image_raw` (width only), and
publish `vision_msgs/Detection3DArray` on `/localizer_node/localized_objects`
with per-object `(x, y)` in `base_link` — bbox centre → bearing through the
62.2° HFOV → windowed-median LiDAR range → planar projection (§2.1). The
provided implementation is the reference and test oracle: swap yours in and the
rest of the stack should behave identically.

Anyone doing this should read §2 first. The window-width decision is the
interesting part of the exercise, not the trigonometry. And subscribe to
`/camera/image_raw` with **BEST_EFFORT** — the default RELIABLE profile will not
match Gazebo's camera and your node will simply never learn the image width
(§7.6).

### 9.2 Fix the nav-adapter frame assumption

While you are in there, write up the known frame simplification in
`tb3_nav_adapter` (§5.1) in one paragraph: what `compute_approach_pose` assumes
about the target's coordinate frame, why it still works in these worlds, and
how you would fix it properly. Hint: a TF lookup of `base_link` in `map` at
query time.

The "why it still works" half is the part worth thinking about. A bug that is
masked by a property of the environment — here, map origin coinciding with the
spawn pose in small rooms — is the kind that ships, and then surfaces the first
time someone runs it in a bigger building.
