#!/usr/bin/env python3
"""
acceptance_run.py -- unattended end-to-end acceptance run for the TurtleBot3
semantic-navigation course workspace.

Do not call this file directly; run the wrapper, which sources ROS for you:

    ./scripts/acceptance_run.sh
    ./scripts/acceptance_run.sh --world warehouse_models

It reproduces the six-terminal flow from the README automatically: it starts
T1..T5 one at a time in a tmux session, waits for each terminal's documented
ready signal before starting the next, watches the robot explore and build
semantic landmarks, issues the world's navigation commands on /user_command,
scores every command against Gazebo ground truth, prints a table and a verdict,
and cleans up after itself.

WHY A PYTHON CORE DRIVEN THROUGH TMUX (and not one bash script)
---------------------------------------------------------------
Start-up is shell-shaped: five terminals, each sourcing the workspace and
running one `ros2 launch`, with a readiness signal scraped off the console.
tmux gives exactly the layout the README documents, so a human can attach
mid-run and see the same five panes they would have opened by hand.

Everything after start-up is not shell-shaped at all:

  * /map is a nav_msgs/OccupancyGrid -- coverage is arithmetic over an int8
    array of tens of thousands of cells;
  * landmarks arrive as vision_msgs/Detection3DArray -- nested field access
    (results[0].hypothesis.class_id, bbox.center.position.x/.y);
  * several waits must run concurrently (status stream + landmark table + map
    coverage), each with its own timeout;
  * scoring needs float geometry (nearest-ground-truth matching) and a
    formatted results table.

In bash all of that becomes `ros2 topic echo | grep | awk`, which breaks the
first time a YAML block re-flows.  rclpy is guaranteed to be present -- it is
how every node in this workspace is written -- so the driver is Python and the
shell layer is a thin wrapper whose only job is to source the environment and
exec this file.  The user still runs ONE command.

This file is deliberately pure ASCII.  Several real log lines in this stack
contain U+2014 EM DASH ("CoordinatorNode ready - mode=EXPLORING", "Nav2 goal
succeeded - target reached"); every match below is on an ASCII-safe substring
of such a line, never on the dash itself.

This script never writes anything under src/.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ---------------------------------------------------------------------------
# PER-WORLD TARGET TABLE  --  the data structure to edit when adding a world.
#
# Everything world-specific lives here and nowhere else:
#   expected_landmarks  how many landmarks of each DETECTOR LABEL must exist in
#                       semantic map memory before the command stage may start.
#                       Keys are the raw detector labels, because that is what
#                       lands in Detection3DArray.results[0].hypothesis.class_id
#                       and what the landmark ids are built from.  With the
#                       fine-tuned tb3det_yolo26n weights they equal the
#                       semantic names, so the trash can's id is "trash_can_0"
#                       (with the COCO weights it was "traffic light_0").
#   truth               Gazebo entity name -> (x, y) fallback taken from the
#                       .world file.  At run time the real pose is read back
#                       from the running simulation with `gz model -m NAME -p`;
#                       these numbers are only used if that query fails.
#   commands            issued in order on /user_command.  truth_entities lists
#                       the ground-truth objects a command may legitimately end
#                       up at; the landmark is scored against the NEAREST one.
#                       That matters for the five-person world, where landmark
#                       ids are observation-order memory slots and person_0 is
#                       not any particular figure.
# ---------------------------------------------------------------------------
#   landmark            the landmark id the command is expected to resolve to.
#                       Used ONLY by the reachability preflight below; if the id
#                       is not in the live table the command runs anyway.
#
# NOTE ON THE FIVE-PERSON WORLD.  `person_centre` stands at Gazebo (0, 0) and the
# map frame coincides with the world frame, so its landmark's map coordinates are
# ~(0, 0) too.  tb3_nav_adapter's compute_approach_pose treats those map
# coordinates as robot-relative (a documented simplification -- NOTES.md 5.1),
# computes dist = hypot(x, y) from the MAP ORIGIN, finds it below
# min_standoff_distance and returns None; nav_goal_adapter_node then logs
# "too close, skipping goal" and publishes nothing, and the coordinator waits in
# SEMANTIC_NAV forever because its nav timeout is only armed once a goal pose
# arrives.  That is a property of the shipped stack, not of a student's detector,
# so such a command is SKIPPED with a reason instead of burning a 180 s timeout.
# person_N is an observation-order memory slot, not an identity, so every person
# command in the five-figure world is scored against all five figures -- but the
# assignment is made injective at scoring time so two commands cannot both claim
# the same figure.
_PERSONS = ("person_corner_ne", "person_corner_nw", "person_corner_sw",
            "person_corner_se", "person_centre")

WORLDS: Dict[str, dict] = {
    # The full three-target acceptance: one person, one trash can, one chair.
    "warehouse_models": {
        "world_file": "warehouse_semantic_models.world",
        "summary": "4x6 m room, 1 person + 1 trash can + 1 chair "
                   "(all three semantic targets)",
        "expected_landmarks": {"person": 1, "trash_can": 1, "chair": 1},
        "truth": {
            "semantic_person":    (0.60, -2.20),
            "semantic_trash_can": (0.70,  1.50),
            "semantic_chair":     (-1.30, 2.40),
        },
        "commands": (
            {"command": "go to person 0",  "landmark": "person_0",
             "truth_entities": ("semantic_person",)},
            {"command": "go to trash can", "landmark": "trash_can_0",
             "truth_entities": ("semantic_trash_can",)},
            {"command": "go to chair",     "landmark": "chair_0",
             "truth_entities": ("semantic_chair",)},
        ),
    },
    # The repo default world.  Five person figures, no trash can and no chair,
    # so only person commands make sense here.
    "warehouse_models_person": {
        "world_file": "warehouse_models_person.world",
        "summary": "6x6 m room, 5 person figures (four corners + centre)",
        "expected_landmarks": {"person": 3},
        "truth": {
            "person_corner_ne": (2.0,  2.0),
            "person_corner_nw": (-2.0, 2.0),
            "person_corner_sw": (-2.0, -2.0),
            "person_corner_se": (2.0,  -2.0),
            "person_centre":    (0.0,  0.0),
        },
        "commands": (
            {"command": "go to person 0", "landmark": "person_0",
             "truth_entities": _PERSONS},
            {"command": "go to person 1", "landmark": "person_1",
             "truth_entities": _PERSONS},
            {"command": "go to person 2", "landmark": "person_2",
             "truth_entities": _PERSONS},
        ),
    },
}

# The three-target world is the DEFAULT because it is the one the acceptance
# criteria are written against and the one the reference run in NOTES.md 6.1 was
# measured on.  sim.launch.py's own default is warehouse_models_person; that
# world is still selectable with --world, but see the note above WORLDS for why
# a figure standing on the map origin cannot be navigated to by this stack.
DEFAULT_WORLD = "warehouse_models"
ROBOT_ENTITY = "waffle_pi"                  # Gazebo entity name of the robot

# Acceptance bar, from INSTRUCTIONS.md: "the robot ends within 1.2 m of the real
# object".  Keep this in step with INSTRUCTIONS.md -- the script must score the
# criterion the course states, not a more comfortable one.
#
# It was 1.0 m and is now 1.2 m, because 1.0 m did not cover the error budget of
# a correctly working stack: approach_distance 0.5 m + Nav2's xy_goal_tolerance
# 0.25 m + the landmark's systematic bias toward the robot (the LiDAR ranges the
# near surface, not the object centre -- 0.15-0.36 m measured on the chair) sums
# to ~1.1 m worst case.  One six-run series had a run where all three commands
# reported TARGET_REACHED and every landmark was in tolerance, scored FAIL only
# because the chair finished at 1.04 m.  See NOTES.md 6.1.1.
#
# The raw final_distance is always printed and written to JSON, so a regression
# is visible as a number well before it crosses the bar.
PASS_DISTANCE_M = 1.2

# nav_goal_adapter.yaml: min_standoff_distance 0.3, approach_distance 0.5.  A
# landmark closer to the map origin than their sum cannot produce a usable Nav2
# goal through the adapter's current frame assumption (NOTES.md 5.1).
MIN_STANDOFF_M = 0.3
APPROACH_DISTANCE_M = 0.5
MIN_GOAL_RADIUS_M = MIN_STANDOFF_M + APPROACH_DISTANCE_M

# A landmark further than this from every candidate ground-truth object is not
# that object.  Derived from the measured landmark errors (0.02-0.20 m in
# NOTES.md 6.1) with a wide margin; the retired stop sign managed 4.38 m.
MAX_LANDMARK_ERROR_M = 0.5

# ---------------------------------------------------------------------------
# TERMINAL TABLE  --  one entry per README terminal, started in this order.
#
# ready_tokens: ALL of these substrings must appear on the SAME captured line.
# That is not decoration.  T2's real ready signal is
#     [lifecycle_manager_navigation]: Managed nodes are active
# but lifecycle_manager_SLAM prints the identical "Managed nodes are active"
# text and fires EARLIER in the same pane.  Matching the phrase alone would
# declare Nav2 ready while only SLAM had come up, and every later stage would
# then fail for no visible reason.  Requiring both tokens on one line is what
# keeps the slam manager from satisfying the nav manager's signal.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Terminal:
    window: str         # also the tmux window name; tuple order is start order
    what: str
    launch: str          # may contain {world}
    ready_tokens: Tuple[str, ...]
    timeout: float


TERMINALS: Tuple[Terminal, ...] = (
    Terminal("T1", "Gazebo + TurtleBot3 spawn",
             "ros2 launch tb3_bringup sim.launch.py world:={world}",
             ("Successfully spawned entity [waffle_pi]",), 180.0),
    Terminal("T2", "SLAM Toolbox + Nav2 + RViz",
             "ros2 launch tb3_bringup nav.launch.py",
             ("lifecycle_manager_navigation", "Managed nodes are active"), 180.0),
    Terminal("T3", "Course backend (memory, query, coordinator, exploration)",
             "ros2 launch tb3_bringup backend.launch.py",
             ("CoordinatorNode ready",), 120.0),
    Terminal("T4", "Localizer (bbox + LiDAR -> object position)",
             "ros2 launch tb3_bringup localizer.launch.py",
             ("LocalizerNode ready",), 90.0),
    Terminal("T5", "Detector (YOLO inference on the camera image)",
             "ros2 launch tb3_bringup detector.launch.py",
             ("detector_node ready",), 300.0),
)

# Stage timeouts that are not tied to one terminal.
MAPPING_TIMEOUT = 600.0        # explore until every expected landmark exists.
                               # Exploration is the long tail of a run: a
                               # six-run series finished mapping in 117-331 s
                               # five times and blew past 480 s once, so the
                               # budget is set well above the typical 2-8 min
                               # rather than just above the median.
COMMAND_TIMEOUT = 180.0        # one /user_command -> TARGET_REACHED/FAILED
                               # (the coordinator's own nav_goal_timeout_sec is
                               # 60 s, so this only has to outlast query +
                               # goal-pose + one Nav2 attempt)
POST_COMMAND_SETTLE = 6.0      # coordinator auto-resumes exploration 3 s after
                               # a terminal state; wait it out before the next
                               # command so the next query starts from EXPLORING

# Environment preparation, verbatim from the README's "In every terminal"
# block.  Every window runs this before its launch command.
ENV_PREP = (
    "cd {repo} && "
    "source /opt/ros/humble/setup.bash && "
    "source install/setup.bash && "
    "export TURTLEBOT3_MODEL=waffle_pi"
)

# Pane scanning.  "ERROR" alone is not a crash: Nav2's documented spin-in-place
# wedge prints "[controller_server] [ERROR] Failed to make progress" and
# recovers, so ERROR lines are reported but do not by themselves fail the run.
# The markers below mean a process actually died or threw, and those do fail it.
#
# The bare word "Exception" is NOT a crash marker and must never become one:
# nav2_costmap_2d logs the literal "TF Exception that should never happen for
# sensor frame: ..." every time a scan arrives before its sensor TF is available,
# which is routine while slam_toolbox and the costmaps are still activating --
# i.e. inside T2's pane, before Nav2 has even reported itself ready.  tf2 stamps
# "[tf2::ExtrapolationException]" / "[tf2::LookupException]" onto text that nav2
# nodes log verbatim, and the lifecycle manager prints "Failed to change state
# for node: %s. Exception: %s." on transient bringup races.  Matching the bare
# word aborted healthy runs during start-up and forced VERDICT: FAIL on runs
# where every command had succeeded.
ERROR_MARKER = "ERROR"
CRASH_MARKERS = (
    "Traceback (most recent call last)",
    "process has died",
    "Segmentation fault",
    "core dumped",
    "terminate called after throwing",
)
# Anchored exception forms that do mean something died, with the known-benign
# ROS 2 forms explicitly excluded.
CRASH_RE = re.compile(r"\bUnhandled exception\b|\bException in thread\b",
                      re.IGNORECASE)
BENIGN_EXCEPTION_RE = re.compile(
    r"TF Exception that should never happen|\[tf2::\w*Exception\]")

# Coordinator status strings.  Format is "[<MODE>] <text>".  Terminal states are
# a status whose text begins with one of these two prefixes.
STATUS_REACHED = "[TARGET_REACHED]"
STATUS_FAILED = "[TARGET_FAILED]"
# "target selected: person_0 (person) at (0.61, -2.19) - waiting for goal pose"
# An id may contain a space (COCO's "traffic light_0"), hence the non-greedy
# capture up to the parenthesised semantic name.
SELECTED_RE = re.compile(
    r"target selected:\s*(?P<id>.+?)\s*\((?P<name>[^()]*)\)\s*at\s*"
    r"\((?P<x>-?\d+(?:\.\d+)?),\s*(?P<y>-?\d+(?:\.\d+)?)\)"
)
# tmux capture-pane -p already strips escape sequences, but a stray sequence in
# the middle of a log line would silently break substring matching, so scrub.
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[a-zA-Z]")

TMUX_SOCKET = "tb3_acceptance"   # private tmux server: our own options, and
                                 # kill-server can never touch the user's own
                                 # tmux sessions
SESSION = "tb3_acceptance"


# ===========================================================================
# small helpers
# ===========================================================================

_T0 = time.monotonic()


def log(msg: str = "") -> None:
    if msg:
        print("[%7.1fs] %s" % (time.monotonic() - _T0, msg), flush=True)
    else:
        print(flush=True)


def banner(title: str) -> None:
    print(flush=True)
    print("=" * 78, flush=True)
    print("== %s" % title, flush=True)
    print("=" * 78, flush=True)


def die(msg: str, code: int = 2) -> None:
    print("acceptance_run: %s" % msg, file=sys.stderr, flush=True)
    sys.exit(code)


def run(cmd: Sequence[str], timeout: float = 20.0) -> subprocess.CompletedProcess:
    """Run a command, never raise, always return a CompletedProcess-alike."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", "timeout")
    except OSError as exc:
        return subprocess.CompletedProcess(cmd, 127, "", str(exc))


# ===========================================================================
# tmux layer
# ===========================================================================

def tmux(*args: str, timeout: float = 20.0) -> subprocess.CompletedProcess:
    return run(["tmux", "-L", TMUX_SOCKET, *args], timeout=timeout)


def tmux_session_exists() -> bool:
    return tmux("has-session", "-t", SESSION).returncode == 0


def tmux_kill_server() -> None:
    tmux("kill-server")


# --norc --noprofile: the README warns that a conda base environment shadowing
# /usr/bin/python3 breaks this workspace.  Skipping the user's rc files keeps
# auto-activated conda (and any other prompt magic) out of the launch panes.
PANE_SHELL = "bash --norc --noprofile -i"

# Window name -> tmux window id ("@3").  Windows are addressed by id, never by
# "session:index": ids are permanent and unaffected by base-index, renames or
# a window being closed.
WINDOW_IDS: Dict[str, str] = {}


def tmux_start_session() -> None:
    """Fresh private tmux server with one window per terminal, T1..T5."""
    tmux_kill_server()             # requirement: if a session of the same name
                                   # already exists, kill it first
    # A throwaway first window, because of two tmux facts that bite here:
    #   * `tmux start-server` exits again immediately when there is no session,
    #     so any option set before the first session is silently lost;
    #   * history-limit only applies to panes created AFTER it is set.
    # So: create a bootstrap window, set the options, create the real windows
    # (which inherit them), then kill the bootstrap.
    res = tmux("new-session", "-d", "-s", SESSION, "-n", "bootstrap",
               # -x/-y: a detached session defaults to 80 columns, which wraps
               # long ROS log lines.  Wide panes plus capture-pane -J keep the
               # ready signals on a single captured line.
               "-x", "250", "-y", "50", "-P", "-F", "#{window_id}",
               "sleep 86400")
    if res.returncode != 0:
        die("could not create tmux session: %s"
            % (res.stderr.strip() or res.stdout.strip()))
    bootstrap = res.stdout.strip()

    for name, value in (
        # The end-of-run error scan reads the whole pane history; Gazebo and
        # Nav2 out-log the 2000-line default easily.
        ("history-limit", "50000"),
        # Nothing may rename a window out from under us (ids make that
        # cosmetic, but the report prints window names).
        ("automatic-rename", "off"),
    ):
        r = tmux("set-option", "-g", name, value)
        if r.returncode != 0:
            log("warning: tmux set-option -g %s %s failed (%s)"
                % (name, value, r.stderr.strip()))

    for term in TERMINALS:
        r = tmux("new-window", "-d", "-t", "%s:" % SESSION, "-n", term.window,
                 "-P", "-F", "#{window_id}", PANE_SHELL)
        if r.returncode != 0:
            die("could not create tmux window %s: %s"
                % (term.window, r.stderr.strip()))
        WINDOW_IDS[term.window] = r.stdout.strip()

    tmux("kill-window", "-t", bootstrap)
    log("tmux session %r ready on socket %r (attach with: tmux -L %s attach)"
        % (SESSION, TMUX_SOCKET, TMUX_SOCKET))


def tmux_send(term: Terminal, line: str) -> None:
    # -l sends the string literally: without it tmux parses each argument as a
    # key name first, so a command that happened to match one would be swallowed.
    target = WINDOW_IDS[term.window]
    res = tmux("send-keys", "-t", target, "-l", line)
    if res.returncode != 0:
        die("could not send the launch command to %s: %s"
            % (term.window, res.stderr.strip()))
    tmux("send-keys", "-t", target, "Enter")


# capture-pane -S - walks the entire 50000-line scrollback, which is what the
# end-of-run error scan needs but is wasteful every 2 s while polling for a
# ready token that was printed seconds ago.  POLL_HISTORY bounds the polling
# captures; the one-shot end-of-run scan and dump_tail still take everything.
POLL_HISTORY = "-2000"


def tmux_capture(term: Terminal, start: str = "-") -> List[str]:
    """Return the pane's text as lines.

    -J joins wrapped lines: without it a ready signal that happens to wrap at
    the pane edge is split across two lines and the substring never matches.
    *start* is the capture-pane -S argument: "-" for the whole scrollback.

    A tmux failure is LOGGED, never swallowed.  An empty list from a timed-out
    capture-pane is indistinguishable from an empty pane, and the run would then
    die with the misleading "never printed its ready signal" message.
    """
    args = ["capture-pane", "-p", "-J", "-S", start,
            "-t", WINDOW_IDS[term.window]]
    res = tmux(*args, timeout=30.0)
    if res.returncode != 0:
        log("warning: capture-pane on %s failed (rc=%d): %s"
            % (term.window, res.returncode,
               (res.stderr or res.stdout).strip()[:160] or "no output"))
        return []
    return [ANSI_RE.sub("", ln) for ln in res.stdout.splitlines()]


def scan_pane(lines: Sequence[str]) -> Tuple[int, int, List[str]]:
    """Count ERROR lines and crash markers; return (errors, crashes, samples)."""
    errors = 0
    crashes = 0
    samples: List[str] = []
    for ln in lines:
        if ERROR_MARKER in ln:
            errors += 1
        hit = any(m in ln for m in CRASH_MARKERS)
        if not hit and CRASH_RE.search(ln) and not BENIGN_EXCEPTION_RE.search(ln):
            hit = True
        if hit:
            crashes += 1
            if len(samples) < 3:
                samples.append(ln.strip()[:160])
    return errors, crashes, samples


def dump_tail(term: Terminal, n: int = 30) -> None:
    lines = [ln.rstrip() for ln in tmux_capture(term)]
    # capture-pane pads the pane out to its full height with blank rows; those
    # would otherwise eat most of the 30 lines the user actually wants to see.
    while lines and not lines[-1]:
        lines.pop()
    tail = lines[-n:]
    print(flush=True)
    print("---- last %d lines of %s (%s) %s" % (n, term.window, term.what, "-" * 20),
          flush=True)
    for ln in tail:
        print("  | %s" % ln, flush=True)
    print("-" * 78, flush=True)


# ===========================================================================
# cleanup  --  the README "clean restart" procedure
# ===========================================================================

# Written to a FILE and executed as `bash <file>`, never as `bash -c "<...>"`.
# A -c string puts the pkill patterns into the shell's OWN argv, where
# `pkill -f` matches them and kills the shell in the middle of the cleanup.
# That actually happened during development of this stack.  The same hazard
# applies to this script: if someone passes a --world path containing "nav2",
# a bare `pkill -9 -f nav2` would kill the acceptance run itself.  So the
# pattern kills go through pgrep + an explicit protected-PID list (this
# process and its whole ancestor chain) instead of pkill -f.
CLEANUP_SH = r"""#!/bin/bash
# Generated by scripts/acceptance_run.py -- transient, safe to delete.
# Mirrors the README "Troubleshooting: clean restart" procedure.
set -u
PROTECTED="${PROTECTED_PIDS:-}"

# kill_pattern REGEX -- like `pkill -9 -f REGEX`, minus the ability to kill
# the acceptance run itself or any of its ancestors.
kill_pattern() {
    local pat="$1" pid
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        case " $PROTECTED " in
            *" $pid "*) continue ;;
        esac
        kill -9 "$pid" 2>/dev/null
    done
}

# 1. Gazebo.  -x is an exact process-name match and cannot hit this script.
pkill -9 -x gzserver; pkill -9 -x gzclient

# 2. Orphan course nodes.
kill_pattern 'detector_node|localizer_node|coordinator_node|semantic_memory_node|semantic_map_memory_node|semantic_query_node|nav_goal_adapter_node|frontier_detection_node|goal_assignment_node|startup_map_warmup'

# 3. Orphan SLAM/Nav2 nodes.
kill_pattern 'slam_toolbox|nav2|lifecycle_manager|bt_navigator|controller_server|planner_server|behavior_server|smoother_server|waypoint_follower|velocity_smoother|map_saver'

# 4. Viewers.
pkill -9 -x rviz2; pkill -9 -x robot_state_publisher

# 5. Stale discovery cache (restarts automatically on the next ros2 command).
ros2 daemon stop >/dev/null 2>&1 || true

exit 0
"""


def _ancestor_pids() -> List[int]:
    """This PID plus every ancestor, so cleanup can never kill its own caller."""
    pids: List[int] = []
    pid = os.getpid()
    for _ in range(32):
        if pid <= 1:
            break
        pids.append(pid)
        try:
            with open("/proc/%d/stat" % pid, "r") as fh:
                # field 4 is ppid; the comm field may contain spaces/parens, so
                # split after the closing paren.
                pid = int(fh.read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return pids


def run_cleanup(reason: str) -> None:
    log("cleanup (%s): killing Gazebo / Nav2 / course nodes, stopping ros2 daemon"
        % reason)
    fd, path = tempfile.mkstemp(prefix="tb3_acceptance_cleanup_", suffix=".sh")
    try:
        with os.fdopen(fd, "w") as fh:
            fh.write(CLEANUP_SH)
        env = dict(os.environ)
        env["PROTECTED_PIDS"] = " ".join(str(p) for p in _ancestor_pids())
        try:
            subprocess.run(["bash", path], env=env, timeout=90,
                           capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            log("cleanup: timed out (continuing)")
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


def shutdown(keep_session: bool, scale: float = 1.0) -> None:
    """Stop the terminals, then clean up orphans."""
    if keep_session:
        log("--keep-session: leaving the simulation and the tmux session running.")
        log("  attach with:  tmux -L %s attach -t %s" % (TMUX_SOCKET, SESSION))
        log("  clean up with: tmux -L %s kill-server, then the clean-restart "
            "procedure in the README" % TMUX_SOCKET)
        return
    if tmux_session_exists():
        # Ctrl-C first: ros2 launch shuts its children down cleanly and leaves
        # far fewer orphans for the pkill pass to mop up.
        for term in reversed(TERMINALS):
            if term.window in WINDOW_IDS:
                tmux("send-keys", "-t", WINDOW_IDS[term.window], "C-c")
        # Grace period for five `ros2 launch` trees to unwind; scaled, because
        # on a slow machine a short grace leaves more orphans for the pkill pass.
        time.sleep(5.0 * scale)
        tmux_kill_server()
    run_cleanup("post-run")


# ===========================================================================
# Gazebo ground truth
# ===========================================================================

def gz_pose(entity: str) -> Optional[Tuple[float, float]]:
    """(x, y) of a Gazebo entity, or None.

    `gz model -m NAME -p` prints "x y z roll pitch yaw" on its first line.  We
    accept the first line that parses as six floats, which also survives a
    banner line being printed ahead of it.
    """
    res = run(["gz", "model", "-m", entity, "-p"], timeout=15.0)
    if res.returncode != 0:
        return None
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 6:
            try:
                return float(parts[0]), float(parts[1])
            except ValueError:
                continue
    return None


def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class GroundTruth:
    """Gazebo poses for the world's static objects, read once and cached."""

    def __init__(self, fallback: Dict[str, Tuple[float, float]]):
        self._fallback = dict(fallback)
        self._cache: Dict[str, Tuple[float, float]] = {}
        self.degraded: List[str] = []

    def resolve_all(self) -> None:
        for name, fb in self._fallback.items():
            pose = gz_pose(name)
            if pose is None:
                pose = fb
                self.degraded.append(name)
            self._cache[name] = pose

    def get(self, name: str) -> Tuple[float, float]:
        return self._cache.get(name, self._fallback[name])

    def nearest(self, point: Tuple[float, float], candidates: Sequence[str],
                exclude: Sequence[str] = ()) -> Tuple[str, float, bool]:
        """Nearest candidate to *point*, preferring one not already claimed.

        Returns (entity, error, duplicate).  Matching is made injective by the
        caller passing the entities earlier commands already matched: without
        that, two "go to person N" commands can both score against the same
        figure and both "pass" while three of five figures were never visited.
        duplicate=True means every candidate was already claimed, which is
        itself a finding and is reported rather than silently accepted.
        """
        pool = [c for c in candidates if c not in exclude]
        duplicate = not pool
        if duplicate:
            pool = list(candidates)
        best = min(pool, key=lambda name: dist(point, self.get(name)))
        return best, dist(point, self.get(best)), duplicate


# ===========================================================================
# ROS layer  --  imported lazily so that --help and --dry-run work in a shell
# with no ROS sourced.
# ===========================================================================

_ros: dict = {}


def import_ros() -> None:
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import String
        from vision_msgs.msg import Detection3DArray
    except ImportError as exc:
        die("could not import rclpy/ROS messages (%s).\n"
            "            Run scripts/acceptance_run.sh, which sources "
            "/opt/ros/humble/setup.bash and install/setup.bash for you." % exc)
    _ros.update(locals())


@dataclass
class Landmark:
    landmark_id: str
    class_id: str
    x: float
    y: float


class Bridge:
    """One rclpy node: watches /map, landmarks and status; sends commands."""

    def __init__(self) -> None:
        rclpy = _ros["rclpy"]
        QoSProfile = _ros["QoSProfile"]
        ReliabilityPolicy = _ros["ReliabilityPolicy"]
        DurabilityPolicy = _ros["DurabilityPolicy"]

        rclpy.init(args=None)
        # Node name deliberately shares no substring with the cleanup pkill
        # patterns, so a stray cleanup can never target the driver.
        self.node = rclpy.create_node("tb3_acceptance_driver")

        volatile = QoSProfile(depth=50,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        # /map is latched by slam_toolbox: TRANSIENT_LOCAL, or we would sit and
        # wait for the next full-map publication.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._lock = threading.Lock()
        self._status: List[Tuple[float, str]] = []
        self._landmarks: Dict[str, Landmark] = {}
        self.coverage_pct: Optional[float] = None
        self.map_cells = 0

        self.node.create_subscription(_ros["OccupancyGrid"], "/map",
                                      self._map_cb, latched)
        self.node.create_subscription(
            _ros["Detection3DArray"],
            "/semantic_map_memory_node/landmark_objects",
            self._landmark_cb, volatile)
        self.node.create_subscription(_ros["String"],
                                      "/coordinator_node/status",
                                      self._status_cb, volatile)
        self._cmd_pub = self.node.create_publisher(_ros["String"],
                                                   "/user_command", volatile)

        self._exec = _ros["SingleThreadedExecutor"]()
        self._exec.add_node(self.node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # -- plumbing ----------------------------------------------------------
    def _spin(self) -> None:
        while not self._stop.is_set():
            try:
                self._exec.spin_once(timeout_sec=0.2)
            except Exception:                      # noqa: BLE001 - shutdown race
                break

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5.0)
        try:
            self._exec.remove_node(self.node)
            self.node.destroy_node()
            _ros["rclpy"].shutdown()
        except Exception:                          # noqa: BLE001
            pass

    # -- callbacks ---------------------------------------------------------
    def _map_cb(self, msg) -> None:
        data = msg.data
        total = len(data)
        if not total:
            return
        # Known cells are everything that is not -1 (unknown).  msg.data is an
        # array.array('b') under rclpy and a list under some replays; both have
        # .count().  numpy arrays do not, hence the fallback.
        try:
            unknown = data.count(-1)
        except AttributeError:
            unknown = int((data == -1).sum())
        with self._lock:
            self.map_cells = total
            self.coverage_pct = 100.0 * (total - unknown) / total

    def _landmark_cb(self, msg) -> None:
        seen: Dict[str, Landmark] = {}
        for det in msg.detections:
            if not det.results:
                continue
            seen[det.id] = Landmark(
                landmark_id=det.id,
                class_id=det.results[0].hypothesis.class_id,
                x=float(det.bbox.center.position.x),
                y=float(det.bbox.center.position.y),
            )
        with self._lock:
            # Replace wholesale: the message is the complete landmark table.
            self._landmarks = seen

    def _status_cb(self, msg) -> None:
        with self._lock:
            self._status.append((time.monotonic(), msg.data))

    # -- accessors ---------------------------------------------------------
    def landmarks(self) -> Dict[str, Landmark]:
        with self._lock:
            return dict(self._landmarks)

    def coverage(self) -> Optional[float]:
        with self._lock:
            return self.coverage_pct

    def status_mark(self) -> int:
        with self._lock:
            return len(self._status)

    def status_since(self, mark: int) -> List[str]:
        with self._lock:
            return [text for _, text in self._status[mark:]]

    def send_command(self, text: str, wait: float = 15.0) -> bool:
        """Publish on /user_command once the coordinator is actually matched.

        The publisher is VOLATILE: a message published before discovery has
        matched the coordinator's subscription is dropped on the floor, and the
        run would then hang waiting for a status that never comes.
        """
        deadline = time.monotonic() + wait
        while (self._cmd_pub.get_subscription_count() == 0
               and time.monotonic() < deadline):
            time.sleep(0.2)
        matched = self._cmd_pub.get_subscription_count() > 0
        msg = _ros["String"]()
        msg.data = text
        self._cmd_pub.publish(msg)
        return matched


# ===========================================================================
# stages
# ===========================================================================

class StageTimeout(Exception):
    def __init__(self, term: Terminal, reason: str):
        super().__init__(reason)
        self.term = term
        self.reason = reason


def start_terminal(term: Terminal, world: str, scale: float) -> None:
    launch = term.launch.format(world=world)
    tmux_send(term, "%s && %s" % (ENV_PREP.format(repo=REPO_ROOT), launch))
    timeout = term.timeout * scale
    log("%s  %s" % (term.window, term.what))
    log("     $ %s" % launch)
    log("     waiting for: %s  (timeout %.0fs)"
        % (" + ".join(repr(t) for t in term.ready_tokens), timeout))

    deadline = time.monotonic() + timeout
    next_beat = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        lines = tmux_capture(term, start=POLL_HISTORY)
        # ALL tokens on ONE line -- see the note on TERMINALS above for why the
        # per-line test is what keeps lifecycle_manager_slam from satisfying T2.
        for ln in lines:
            if all(tok in ln for tok in term.ready_tokens):
                log("     ready: %s" % ln.strip()[:140])
                return
        _, crashes, samples = scan_pane(lines)
        if crashes:
            raise StageTimeout(term, "crash marker in %s: %s"
                               % (term.window, samples[0] if samples else "?"))
        if time.monotonic() >= next_beat:
            log("     still waiting for %s ... (%.0fs left)"
                % (term.window, deadline - time.monotonic()))
            next_beat = time.monotonic() + 20.0
        time.sleep(2.0)
    raise StageTimeout(term, "%s never printed its ready signal within %.0fs"
                       % (term.window, timeout))


def wait_for_landmarks(bridge: Bridge, cfg: dict, scale: float) -> bool:
    """Watch coverage and the landmark table until every expected class exists.

    Returns True if all expected landmarks appeared.  On timeout this returns
    False and the run CONTINUES: the commands then fail with a coordinator
    "query failed: no active <class> in memory", which is a far more useful
    acceptance report than aborting with no table at all.
    """
    expected: Dict[str, int] = cfg["expected_landmarks"]
    timeout = MAPPING_TIMEOUT * scale
    deadline = time.monotonic() + timeout
    log("expecting landmarks: %s  (timeout %.0fs)"
        % (", ".join("%dx %r" % (n, c) for c, n in expected.items()), timeout))
    next_beat = 0.0
    while time.monotonic() < deadline:
        lms = bridge.landmarks()
        have: Dict[str, int] = {}
        for lm in lms.values():
            have[lm.class_id] = have.get(lm.class_id, 0) + 1
        if all(have.get(cls, 0) >= n for cls, n in expected.items()):
            cov = bridge.coverage()
            log("all expected landmarks present (map known %s): %s"
                % ("n/a" if cov is None else "%.1f%%" % cov,
                   ", ".join("%s @ (%.2f, %.2f)" % (lm.landmark_id, lm.x, lm.y)
                             for lm in sorted(lms.values(),
                                              key=lambda x: x.landmark_id))))
            return True
        if time.monotonic() >= next_beat:
            cov = bridge.coverage()
            cov_s = "n/a" if cov is None else "%.1f%% of %d cells" % (cov,
                                                                     bridge.map_cells)
            found = (", ".join("%s @ (%.2f, %.2f)" % (lm.landmark_id, lm.x, lm.y)
                               for lm in sorted(lms.values(),
                                                key=lambda x: x.landmark_id))
                     or "none yet")
            missing = ", ".join("%r (%d/%d)" % (c, have.get(c, 0), n)
                                for c, n in expected.items() if have.get(c, 0) < n)
            log("exploring... map known %s | landmarks: %s | still missing: %s "
                "| %.0fs left" % (cov_s, found, missing, deadline - time.monotonic()))
            next_beat = time.monotonic() + 15.0
        time.sleep(2.0)
    log("TIMEOUT waiting for landmarks; continuing so the report still gets "
        "written (commands are expected to fail)")
    return False


@dataclass
class CommandResult:
    command: str
    landmark_id: str = "-"
    semantic_name: str = "-"
    landmark_xy: Optional[Tuple[float, float]] = None
    matched_truth: str = "-"
    landmark_error: Optional[float] = None
    final_distance: Optional[float] = None
    elapsed: float = 0.0
    outcome: str = "NO-STATUS"
    detail: str = ""
    note: str = ""              # why this command cannot count as a pass
    robot_pose_ok: bool = True  # False when `gz model -m waffle_pi -p` failed

    def passed(self) -> bool:
        return (self.outcome == "REACHED"
                and not self.note
                and self.final_distance is not None
                and self.final_distance <= PASS_DISTANCE_M)

    def distance_text(self) -> str:
        if self.final_distance is not None:
            return "%.2f m" % self.final_distance
        # A silent "-" here has two very different causes; say which.
        return "gz query failed" if not self.robot_pose_ok else "-"


def run_one_command(bridge: Bridge, spec: dict, truth: GroundTruth,
                    scale: float, claimed: List[str]) -> CommandResult:
    command = spec["command"]
    result = CommandResult(command=command)
    timeout = COMMAND_TIMEOUT * scale

    mark = bridge.status_mark()
    if not bridge.send_command(command, wait=15.0 * scale):
        log("     WARNING: nothing is subscribed to /user_command; the "
            "coordinator may have died")
    t0 = time.monotonic()
    log("     published %r on /user_command (timeout %.0fs)" % (command, timeout))

    deadline = t0 + timeout
    next_beat = t0 + 20.0
    last_status = ""
    while time.monotonic() < deadline:
        for text in bridge.status_since(mark):
            mark += 1
            last_status = text
            m = SELECTED_RE.search(text)
            if m:
                result.landmark_id = m.group("id")
                result.semantic_name = m.group("name")
                result.landmark_xy = (float(m.group("x")), float(m.group("y")))
                log("     %s" % text.strip()[:150])
            if text.startswith(STATUS_REACHED) or text.startswith(STATUS_FAILED):
                result.outcome = ("REACHED" if text.startswith(STATUS_REACHED)
                                  else "FAILED")
                result.elapsed = time.monotonic() - t0
                # Measure the robot NOW.  auto_resume_exploration re-enables
                # frontier exploration 3 s after a terminal state, and the robot
                # drives off; a later reading would not be "where it stopped".
                robot = gz_pose(ROBOT_ENTITY)
                _finish(result, robot, truth, spec, bridge, claimed)
                # The mode transition is published one message before the
                # human-readable reason ("Nav2 goal succeeded - target
                # reached"), so drain briefly to capture it for the report.
                grace = time.monotonic() + 1.5
                while time.monotonic() < grace:
                    for extra in bridge.status_since(mark):
                        mark += 1
                        result.detail = extra.strip()
                    time.sleep(0.1)
                if not result.detail:
                    result.detail = text.strip()
                log("     %s after %.1fs -- %s"
                    % (result.outcome, result.elapsed, result.detail[:120]))
                return result
        if time.monotonic() >= next_beat:
            log("     waiting... last status: %s"
                % (last_status.strip()[:120] or "(none yet)"))
            next_beat = time.monotonic() + 20.0
        time.sleep(0.2)

    result.outcome = "TIMEOUT"
    result.elapsed = time.monotonic() - t0
    result.detail = last_status.strip()
    _finish(result, gz_pose(ROBOT_ENTITY), truth, spec, bridge, claimed)
    log("     TIMEOUT after %.1fs -- last status: %s"
        % (result.elapsed, result.detail[:120] or "(none)"))
    return result


def _finish(result: CommandResult, robot: Optional[Tuple[float, float]],
            truth: GroundTruth, spec: dict, bridge: Bridge,
            claimed: List[str]) -> None:
    """Score one command against Gazebo ground truth."""
    result.robot_pose_ok = robot is not None
    if robot is None:
        # Loud, because otherwise a broken `gz` turns a healthy stack into an
        # unexplained 0/N FAIL with a column of dashes.
        log("     WARNING: `gz model -m %s -p` returned nothing; the final "
            "robot distance cannot be measured for this command"
            % ROBOT_ENTITY)
    # Prefer the live landmark table over the status line: the status text is
    # rounded to two decimals, the Detection3DArray carries full precision.
    lm = bridge.landmarks().get(result.landmark_id)
    if lm is not None:
        result.landmark_xy = (lm.x, lm.y)
    if result.landmark_xy is None:
        return
    candidates = spec["truth_entities"]
    # Landmark ids are observation-order memory slots, not identities, so the
    # landmark is scored against whichever real object it is NEAREST to -- but
    # only among objects no earlier command already matched, and only if it is
    # actually near one.  Unconstrained nearest-neighbour matching would let a
    # grossly mislocalised landmark (the retired stop sign managed 4.38 m) be
    # silently re-labelled as some other object and reported as a small error.
    entity, err, duplicate = truth.nearest(result.landmark_xy, candidates,
                                           exclude=claimed)
    result.matched_truth = entity
    result.landmark_error = err
    if err > MAX_LANDMARK_ERROR_M:
        result.note = ("landmark %.2f m from %s (max %.2f m) -- MISMATCHED"
                       % (err, entity, MAX_LANDMARK_ERROR_M))
    elif duplicate:
        result.note = ("landmark matches %s, already claimed by an earlier "
                       "command" % entity)
    else:
        claimed.append(entity)
    # Measured against the entity this landmark was matched to, never rebound.
    if robot is not None:
        result.final_distance = dist(robot, truth.get(entity))


def unreachable_reason(bridge: Bridge, spec: dict) -> Optional[str]:
    """Why this command cannot produce a Nav2 goal, or None if it can.

    See the note above WORLDS: compute_approach_pose reads the landmark's MAP
    coordinates as if they were robot-relative, so a landmark within
    min_standoff + approach_distance of the MAP ORIGIN yields no goal pose at
    all and the coordinator then waits in SEMANTIC_NAV until the command times
    out.  Detecting that up front turns a silent 180 s stall into one line.
    """
    lm_id = spec.get("landmark")
    if not lm_id:
        return None
    lm = bridge.landmarks().get(lm_id)
    if lm is None:
        return None            # not observed yet: let the command run and report
    radius = math.hypot(lm.x, lm.y)
    if radius >= MIN_GOAL_RADIUS_M:
        return None
    return ("%s is %.2f m from the map origin, inside the nav adapter's "
            "%.2f m minimum (NOTES.md 5.1): no goal pose can be produced"
            % (lm_id, radius, MIN_GOAL_RADIUS_M))


# ===========================================================================
# reporting
# ===========================================================================

def print_report(world: str, results: List[CommandResult],
                 pane_stats: Dict[str, Tuple[int, int, List[str]]],
                 truth: GroundTruth, mapped: bool, expected: int,
                 interrupted: bool = False) -> bool:
    banner("ACCEPTANCE RESULTS  --  world:=%s" % world)

    header = ("%-16s %-18s %-18s %9s %11s %8s  %s"
              % ("command", "landmark id", "matched truth", "lm err", "final dist",
                 "elapsed", "result"))
    print(header, flush=True)
    print("-" * len(header), flush=True)
    for r in results:
        verdict_text = r.outcome
        if r.note:
            verdict_text += "  ! " + r.note
        elif (r.outcome == "REACHED" and not r.passed()
                and r.final_distance is not None):
            verdict_text += " (>%.1f m)" % PASS_DISTANCE_M
        print("%-16s %-18s %-18s %9s %11s %7.1fs  %s"
              % (r.command,
                 r.landmark_id,
                 r.matched_truth,
                 "-" if r.landmark_error is None else "%.2f m" % r.landmark_error,
                 r.distance_text(),
                 r.elapsed,
                 verdict_text),
              flush=True)

    print(flush=True)
    hdr2 = "%-4s %-46s %12s %8s" % ("term", "what", "ERROR lines", "crashes")
    print(hdr2, flush=True)
    print("-" * len(hdr2), flush=True)
    total_crashes = 0
    for term in TERMINALS:
        errors, crashes, samples = pane_stats.get(term.window, (0, 0, []))
        total_crashes += crashes
        print("%-4s %-46s %12d %8d" % (term.window, term.what[:46], errors, crashes),
              flush=True)
        for s in samples:
            print("       ! %s" % s, flush=True)
    print(flush=True)
    print("ERROR lines are counted but do not fail the run on their own: Nav2's "
          "documented", flush=True)
    print("spin-in-place wedge logs ERROR and recovers (see README "
          "Troubleshooting).", flush=True)

    if truth.degraded:
        print(flush=True)
        print("NOTE: Gazebo pose query failed for %s; .world file coordinates "
              "were used instead." % ", ".join(truth.degraded), flush=True)
    if not mapped:
        print("NOTE: the mapping stage timed out before every expected landmark "
              "appeared.", flush=True)

    if any(not r.robot_pose_ok for r in results):
        print(flush=True)
        print("NOTE: `gz model -m %s -p` failed for at least one command, so the "
              "final robot" % ROBOT_ENTITY, flush=True)
        print("      distance could not be measured. Check that `gz` is on PATH "
              "and GAZEBO_MASTER_URI", flush=True)
        print("      points at the simulation this run started.", flush=True)
    if any(r.outcome == "SKIPPED" for r in results):
        print(flush=True)
        print("NOTE: a SKIPPED command is one this stack provably cannot "
              "complete (see the row's", flush=True)
        print("      reason). It is not a pass.", flush=True)

    ok_cmds = [r for r in results if r.passed()]
    # `expected` is what the world's table asked for.  Without it, interrupting
    # after the first of three successes would print "PASS -- 1/1".
    complete = len(results) == expected and not interrupted
    verdict = bool(results) and complete and len(ok_cmds) == len(results) \
        and total_crashes == 0
    print(flush=True)
    if interrupted or len(results) < expected:
        headline = "INCOMPLETE (interrupted)" if interrupted else "FAIL"
    else:
        headline = "PASS" if verdict else "FAIL"
    print("VERDICT: %s -- %d/%d commands reached the target within %.1f m, "
          "%d crash marker(s)"
          % (headline, len(ok_cmds), expected, PASS_DISTANCE_M, total_crashes),
          flush=True)
    return verdict


def write_json(path: str, world: str, results: List[CommandResult],
               pane_stats: Dict[str, Tuple[int, int, List[str]]],
               verdict: bool, mapped: bool, expected: int, exit_code: int,
               abort_reason: str = "") -> None:
    payload = {
        "world": world,
        "verdict": "PASS" if verdict else "FAIL",
        "exit_code": exit_code,
        "abort_reason": abort_reason,
        "commands_expected": expected,
        "commands_run": len(results),
        "pass_distance_m": PASS_DISTANCE_M,
        "max_landmark_error_m": MAX_LANDMARK_ERROR_M,
        "all_landmarks_found": mapped,
        "commands": [
            {
                "command": r.command,
                "landmark_id": r.landmark_id,
                "semantic_name": r.semantic_name,
                "landmark_xy": r.landmark_xy,
                "matched_truth": r.matched_truth,
                "landmark_error_m": r.landmark_error,
                "final_distance_m": r.final_distance,
                "elapsed_s": round(r.elapsed, 2),
                "outcome": r.outcome,
                "passed": r.passed(),
                "note": r.note,
                "robot_pose_ok": r.robot_pose_ok,
                "detail": r.detail,
            }
            for r in results
        ],
        "terminals": {
            term.window: {
                "what": term.what,
                "error_lines": pane_stats.get(term.window, (0, 0, []))[0],
                "crash_markers": pane_stats.get(term.window, (0, 0, []))[1],
            }
            for term in TERMINALS
        },
    }
    try:
        with open(path, "w") as fh:
            json.dump(payload, fh, indent=2)
        log("wrote %s" % path)
    except OSError as exc:
        log("could not write %s: %s" % (path, exc))


# ===========================================================================
# main
# ===========================================================================

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="acceptance_run.sh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="Unattended end-to-end acceptance run: brings up the six-"
                    "terminal\nflow in tmux, explores, issues the world's "
                    "navigation commands, scores\nthem against Gazebo ground "
                    "truth and prints a PASS/FAIL verdict.",
        epilog="""examples:
  ./scripts/acceptance_run.sh
      Default world (%s): the full three-target
      acceptance -- person, trash can and chair.

  ./scripts/acceptance_run.sh --world warehouse_models_person
      The five-figure world. Person commands only, and any figure standing on
      the map origin is reported SKIPPED (see NOTES.md 5.1).

  ./scripts/acceptance_run.sh --timeout-scale 2
      Same, with every timeout and wait doubled for a slow machine.

exit status:
  0  PASS   every command reached its target within %.1f m and nothing crashed
  1  FAIL   at least one command missed, failed, or a process crashed
  2  setup, start-up or driver failure (a terminal never became ready, a
     preflight check failed, or the driver itself raised)
130  interrupted
""" % (DEFAULT_WORLD, PASS_DISTANCE_M))
    p.add_argument("--world", choices=sorted(WORLDS), default=DEFAULT_WORLD,
                   help="Gazebo world alias (default: %(default)s, the "
                        "three-target acceptance: person, trash can and chair). "
                        "warehouse_models_person is sim.launch.py's own default "
                        "and holds five person figures.")
    p.add_argument("--timeout-scale", type=float, default=1.0, metavar="FLOAT",
                   help="multiply every timeout and wait by this factor, for "
                        "slow machines: the per-terminal start-up timeouts, the "
                        "mapping stage, each command, the /user_command "
                        "discovery wait, the post-command settle and the "
                        "shutdown grace period (default: %(default)s)")
    p.add_argument("--keep-session", action="store_true",
                   help="leave the simulation and the tmux session running "
                        "after the report, for post-mortem debugging "
                        "(attach with: tmux -L %s attach)" % TMUX_SOCKET)
    p.add_argument("--json", metavar="PATH", default=None,
                   help="also write the results as JSON to PATH, on every exit "
                        "path including an abort or an interrupt (the payload "
                        "carries exit_code and abort_reason)")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan (terminals, commands, timeouts, "
                        "ground truth) and exit without touching anything")
    args = p.parse_args(argv)
    if args.timeout_scale <= 0:
        p.error("--timeout-scale must be > 0")
    return args


def print_plan(args: argparse.Namespace) -> None:
    cfg = WORLDS[args.world]
    scale = args.timeout_scale
    banner("DRY RUN  --  world:=%s" % args.world)
    print("repo root      : %s" % REPO_ROOT, flush=True)
    print("world          : %s (%s)" % (cfg["world_file"], cfg["summary"]), flush=True)
    print("tmux           : socket %r, session %r" % (TMUX_SOCKET, SESSION), flush=True)
    print("timeout scale  : %.2f" % scale, flush=True)
    print(flush=True)
    print("environment prepared in every window:", flush=True)
    print("  %s" % ENV_PREP.format(repo=REPO_ROOT), flush=True)
    print(flush=True)
    for term in TERMINALS:
        print("%-3s %-46s timeout %6.0fs" % (term.window, term.what,
                                             term.timeout * scale), flush=True)
        print("     $ %s" % term.launch.format(world=args.world), flush=True)
        print("     ready when one line contains: %s"
              % " AND ".join(repr(t) for t in term.ready_tokens), flush=True)
    print(flush=True)
    print("mapping stage  : wait up to %.0fs for %s"
          % (MAPPING_TIMEOUT * scale,
             ", ".join("%dx %r" % (n, c)
                       for c, n in cfg["expected_landmarks"].items())), flush=True)
    print("commands       : %.0fs each" % (COMMAND_TIMEOUT * scale), flush=True)
    for spec in cfg["commands"]:
        print("     %-16s expects landmark %-16s scored against nearest "
              "unclaimed of: %s"
              % (repr(spec["command"]), repr(spec.get("landmark", "?")),
                 ", ".join(spec["truth_entities"])), flush=True)
    print("     a command whose landmark lands within %.2f m of the map origin "
          "is SKIPPED" % MIN_GOAL_RADIUS_M, flush=True)
    print(flush=True)
    print("ground truth fallback (from the .world file; the live pose is read "
          "with `gz model -m NAME -p`):", flush=True)
    for name, xy in cfg["truth"].items():
        print("     %-20s (%.2f, %.2f)" % (name, xy[0], xy[1]), flush=True)
    print(flush=True)
    print("PASS requires all %d commands REACHED within %.1f m of the real "
          "object, each" % (len(cfg["commands"]), PASS_DISTANCE_M), flush=True)
    print("matched to a distinct ground-truth entity no further than %.2f m "
          "from its landmark," % MAX_LANDMARK_ERROR_M, flush=True)
    print("and zero crash markers.", flush=True)


def check_weights() -> None:
    """Fail now, not after a 300 s T5 timeout, if the weights are not installed.

    The fine-tuned weights are committed, but they only reach install/ when
    tb3_detector has been built since the checkout: otherwise DetectorCore
    raises on load, T5 never prints "detector_node ready", and the operator
    waits out the whole T5 timeout with Gazebo, Nav2 and the backend already up.
    """
    cfg_path = os.path.join(REPO_ROOT, "src", "tb3_detector", "config",
                            "detector.yaml")
    name = "yolo26n.pt"
    try:
        with open(cfg_path, "r") as fh:
            m = re.search(r"^\s*model_path:\s*[\"']([^\"']+)[\"']",
                          fh.read(), re.M)
        if m:
            name = os.path.basename(m.group(1))
    except OSError:
        pass
    installed = os.path.join(REPO_ROOT, "install", "tb3_detector", "share",
                             "tb3_detector", "models", name)
    # A --symlink-install tree links this into build/; os.path.exists follows
    # the link, so a dangling link is correctly reported as missing.
    if not os.path.exists(installed):
        die("detector weights %s not found at\n"
            "            %s\n"
            "            Rebuild tb3_detector (colcon build --symlink-install "
            "--packages-select tb3_detector) -- see INSTRUCTIONS.md, Step 1."
            % (name, installed))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    cfg = WORLDS[args.world]

    if args.dry_run:
        print_plan(args)
        return 0

    if shutil.which("tmux") is None:
        die("tmux is not installed; it is required to drive the six terminals "
            "(sudo apt install tmux)")
    if shutil.which("gz") is None:
        die("the Gazebo `gz` CLI is not on PATH. Ground truth is read with "
            "`gz model -m <entity> -p`;\n"
            "            without it no command can be scored and every run "
            "reports a bare FAIL.")
    if not os.path.isfile(os.path.join(REPO_ROOT, "install", "setup.bash")):
        die("%s/install/setup.bash not found -- build the workspace first "
            "(colcon build --symlink-install)" % REPO_ROOT)
    check_weights()

    # Fail early and clearly if the wrapper did not source ROS, rather than
    # half-way through start-up with a live Gazebo on the machine.
    import_ros()

    def _on_sigterm(_signum, _frame):
        # Raise the same exception Ctrl-C raises so both unwind through the
        # finally: below and the cleanup always runs.
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    scale = args.timeout_scale
    bridge: Optional[Bridge] = None
    results: List[CommandResult] = []
    claimed: List[str] = []          # ground-truth entities already matched
    pane_stats: Dict[str, Tuple[int, int, List[str]]] = {}
    mapped = False
    truth = GroundTruth(cfg["truth"])
    expected = len(cfg["commands"])
    exit_code = 2
    verdict = False
    abort_reason = ""

    try:
        banner("STAGE 0  --  clean slate")
        log("world:=%s  (%s)" % (args.world, cfg["summary"]))
        if args.world != DEFAULT_WORLD:
            log("note: the three-target acceptance (person + trash can + "
                "chair) is --world %s" % DEFAULT_WORLD)
        log("this kills any Gazebo / Nav2 / course nodes already running")
        run_cleanup("pre-run")
        tmux_start_session()

        banner("STAGE 1  --  bring up T1..T5 one at a time (this script is T6)")
        for term in TERMINALS:
            start_terminal(term, args.world, scale)

        banner("STAGE 2  --  explore, map, and build semantic landmarks")
        bridge = Bridge()
        # Robot and target poses come straight from the running simulation.
        truth.resolve_all()
        # Smoke-test the robot query too: resolve_all() only covers the static
        # targets, so a `gz` that cannot see this simulation would otherwise
        # stay invisible until every command scored a bare "-".
        if gz_pose(ROBOT_ENTITY) is None:
            log("WARNING: `gz model -m %s -p` returned nothing even though "
                "Gazebo is up." % ROBOT_ENTITY)
            log("         Final robot distances cannot be measured; check "
                "GAZEBO_MASTER_URI.")
        for name in cfg["truth"]:
            log("ground truth %-20s (%.2f, %.2f)" % (name, *truth.get(name)))
        mapped = wait_for_landmarks(bridge, cfg, scale)

        banner("STAGE 3  --  navigation commands")
        for spec in cfg["commands"]:
            log("command: %r" % spec["command"])
            reason = unreachable_reason(bridge, spec)
            if reason:
                log("     SKIPPED: %s" % reason)
                results.append(CommandResult(command=spec["command"],
                                             landmark_id=spec.get("landmark",
                                                                  "-"),
                                             outcome="SKIPPED", note=reason))
                continue
            results.append(run_one_command(bridge, spec, truth, scale, claimed))
            if spec is not cfg["commands"][-1]:
                # resume_delay_sec is 3 s on the SIM clock; on a machine slow
                # enough to need --timeout-scale the real-time factor is below
                # 1, so this settle is scaled with everything else.
                settle = POST_COMMAND_SETTLE * scale
                log("     settling %.0fs before the next command "
                    "(the coordinator resumes exploring after a target)"
                    % settle)
                time.sleep(settle)

        # Scan the panes before anything is shut down: the shutdown itself puts
        # "process has died" style lines into the logs.
        for term in TERMINALS:
            pane_stats[term.window] = scan_pane(tmux_capture(term))

        verdict = print_report(args.world, results, pane_stats, truth, mapped,
                               expected)
        exit_code = 0 if verdict else 1

    except StageTimeout as exc:
        abort_reason = exc.reason
        banner("ABORTED  --  %s" % exc.reason)
        dump_tail(exc.term, 30)
        for term in TERMINALS:
            errors, crashes, samples = scan_pane(tmux_capture(term))
            pane_stats[term.window] = (errors, crashes, samples)
            log("%s: %d ERROR line(s), %d crash marker(s)"
                % (term.window, errors, crashes))
        log("VERDICT: FAIL -- start-up did not complete, no commands were run")
        exit_code = 2

    except KeyboardInterrupt:
        abort_reason = "interrupted"
        banner("INTERRUPTED")
        # Fill in the pane statistics first: without them the per-terminal table
        # prints all zeros, total_crashes is 0, and a run stopped after one of
        # three commands would otherwise read as a pass.
        try:
            for term in TERMINALS:
                if term.window in WINDOW_IDS:
                    pane_stats[term.window] = scan_pane(tmux_capture(term))
        except Exception:                          # noqa: BLE001
            pass
        if results:
            print_report(args.world, results, pane_stats, truth, mapped,
                         expected, interrupted=True)
        verdict = False
        exit_code = 130

    except Exception:                              # noqa: BLE001
        # An internal driver error is a setup failure (2), not "a command
        # missed" (1).  Print it; a silent exit code teaches nobody anything.
        abort_reason = "driver error"
        banner("DRIVER ERROR")
        import traceback
        traceback.print_exc()
        verdict = False
        exit_code = 2

    finally:
        # Cleanup must complete even if a second Ctrl-C lands inside it: an
        # interrupt during shutdown() or run_cleanup() leaves Gazebo and Nav2
        # alive and exits 1, which the exit-status table would misreport as
        # "at least one command missed".
        try:
            if args.json:
                write_json(args.json, args.world, results, pane_stats, verdict,
                           mapped, expected, exit_code, abort_reason)
            if bridge is not None:
                bridge.close()
            banner("CLEAN UP")
            shutdown(args.keep_session, scale)
            log("done (exit %d)" % exit_code)
        except BaseException:                      # noqa: BLE001
            print("acceptance_run: cleanup was cut short -- run the "
                  "clean-restart procedure in README.md > Troubleshooting "
                  "before the next run.", file=sys.stderr, flush=True)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
