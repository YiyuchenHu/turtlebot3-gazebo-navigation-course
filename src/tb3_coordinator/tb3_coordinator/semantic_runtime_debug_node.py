#!/usr/bin/env python3
"""
semantic_runtime_debug_node.py — Lightweight runtime diagnostics for person vs bench perception.

Subscribes to the detector, localizer, and memory topics without modifying any
existing node.  Periodically prints a concise summary and optionally writes a
CSV log for offline analysis.

Launch together with full_semantic_nav.launch.py:
    ros2 launch tb3_coordinator full_semantic_nav.launch.py use_runtime_debug:=true
"""

import csv
import math
import os
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray


class SemanticRuntimeDebugNode(Node):

    FOCUS_CLASSES = {"bench", "person"}

    def __init__(self):
        super().__init__("semantic_runtime_debug_node")

        self.declare_parameter("summary_interval_sec", 5.0)
        self.declare_parameter("csv_log_enabled", True)
        self.declare_parameter("csv_log_dir", "/tmp/semantic_debug")
        self.declare_parameter("detections_topic", "/detector_node/detections")
        self.declare_parameter("debug_image_topic", "/detector_node/debug_image")
        self.declare_parameter("localized_topic", "/localizer_node/localized_objects")
        self.declare_parameter("memory_topic", "/semantic_memory_node/objects")
        self.declare_parameter("camera_topic", "/camera/image_raw")

        det_topic = self.get_parameter("detections_topic").value
        dbg_topic = self.get_parameter("debug_image_topic").value
        loc_topic = self.get_parameter("localized_topic").value
        mem_topic = self.get_parameter("memory_topic").value
        cam_topic = self.get_parameter("camera_topic").value

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            Detection2DArray, det_topic, self._cb_detections, reliable_qos)
        self.create_subscription(
            Image, dbg_topic, self._cb_debug_image, best_effort_qos)
        self.create_subscription(
            Detection3DArray, loc_topic, self._cb_localized, reliable_qos)
        self.create_subscription(
            Detection3DArray, mem_topic, self._cb_memory, reliable_qos)
        self.create_subscription(
            Image, cam_topic, self._cb_camera, best_effort_qos)

        self._det_count = defaultdict(int)
        self._det_conf_sum = defaultdict(float)
        self._loc_count = defaultdict(int)
        self._mem_active = defaultdict(int)
        self._confusion = defaultdict(int)

        self._cam_times = []
        self._det_times = []
        self._dbg_times = []
        self._loc_times = []

        self._summary_n = 0
        self._start_wall = time.monotonic()

        interval = self.get_parameter("summary_interval_sec").value
        self.create_timer(interval, self._print_summary)

        self._csv_writer = None
        self._csv_file = None
        if self.get_parameter("csv_log_enabled").value:
            log_dir = self.get_parameter("csv_log_dir").value
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.join(log_dir, f"semantic_debug_{ts}.csv")
            self._csv_file = open(csv_path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "wall_time", "source", "class", "confidence",
                "bbox_cx", "bbox_cy", "bbox_w", "bbox_h",
                "x", "y", "range_m", "bearing_deg",
            ])
            self.get_logger().info(f"CSV log -> {csv_path}")

    def destroy_node(self):
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()

    # -- Callbacks --

    def _cb_camera(self, msg: Image):
        self._cam_times.append(time.monotonic())
        self._trim(self._cam_times)

    def _cb_debug_image(self, msg: Image):
        self._dbg_times.append(time.monotonic())
        self._trim(self._dbg_times)

    def _cb_detections(self, msg: Detection2DArray):
        now = time.monotonic()
        self._det_times.append(now)
        self._trim(self._det_times)

        labels_this_frame = []
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            label = best.hypothesis.class_id
            conf = best.hypothesis.score
            labels_this_frame.append(label)

            self._det_count[label] += 1
            self._det_conf_sum[label] += conf

            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y

            if self._csv_writer:
                self._csv_writer.writerow([
                    f"{now:.3f}", "det", label, f"{conf:.3f}",
                    f"{cx:.1f}", f"{cy:.1f}", f"{bw:.1f}", f"{bh:.1f}",
                    "", "", "", "",
                ])

        self._check_confusion(labels_this_frame)

    def _cb_localized(self, msg: Detection3DArray):
        now = time.monotonic()
        self._loc_times.append(now)
        self._trim(self._loc_times)

        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            label = best.hypothesis.class_id
            conf = best.hypothesis.score
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            rng = math.hypot(x, y)
            bearing = math.degrees(math.atan2(-y, x)) if rng > 0.01 else 0.0

            self._loc_count[label] += 1

            if self._csv_writer:
                self._csv_writer.writerow([
                    f"{now:.3f}", "loc", label, f"{conf:.3f}",
                    "", "", "", "",
                    f"{x:.3f}", f"{y:.3f}", f"{rng:.3f}", f"{bearing:.1f}",
                ])

    def _cb_memory(self, msg: Detection3DArray):
        snapshot = defaultdict(int)
        for det in msg.detections:
            if not det.results:
                continue
            label = max(det.results, key=lambda r: r.hypothesis.score).hypothesis.class_id
            snapshot[label] += 1
        self._mem_active = snapshot

    # -- Confusion analysis --

    def _check_confusion(self, labels):
        n_person = labels.count("person")
        n_bench = labels.count("bench")
        has_bench = n_bench > 0
        has_person = n_person > 0

        if has_person and not has_bench and n_person >= 2:
            self._confusion["bench_as_person"] += 1
        if has_bench and not has_person and n_bench >= 2:
            self._confusion["person_as_bench"] += 1

    # -- Periodic summary --

    def _print_summary(self):
        self._summary_n += 1
        elapsed = time.monotonic() - self._start_wall

        cam_fps = self._fps(self._cam_times)
        det_fps = self._fps(self._det_times)
        dbg_fps = self._fps(self._dbg_times)
        loc_fps = self._fps(self._loc_times)

        det_interval = self._avg_interval(self._det_times)
        dbg_interval = self._avg_interval(self._dbg_times)
        dbg_lag = ""
        if det_interval > 0 and dbg_interval > 0:
            ratio = dbg_interval / det_interval
            if ratio > 2.0:
                dbg_lag = f" WARNING debug_image {ratio:.1f}x slower than detections"

        lines = [
            f"=== Semantic Debug #{self._summary_n}  t={elapsed:.0f}s ===",
            f"  FPS  cam={cam_fps:.1f}  det={det_fps:.1f}  dbg_img={dbg_fps:.1f}  loc={loc_fps:.1f}{dbg_lag}",
        ]

        for cls in ["person", "bench"]:
            n = self._det_count.get(cls, 0)
            avg_c = (self._det_conf_sum[cls] / n) if n > 0 else 0.0
            n_loc = self._loc_count.get(cls, 0)
            n_mem = self._mem_active.get(cls, 0)
            lines.append(
                f"  {cls:8s}  det={n:4d}  avg_conf={avg_c:.2f}  "
                f"loc={n_loc:4d}  mem_active={n_mem}"
            )

        other_classes = set(self._det_count.keys()) - self.FOCUS_CLASSES
        if other_classes:
            others = ", ".join(
                f"{c}={self._det_count[c]}" for c in sorted(other_classes)
            )
            lines.append(f"  other_det: {others}")

        if any(v > 0 for v in self._confusion.values()):
            parts = [f"{k}={v}" for k, v in sorted(self._confusion.items()) if v > 0]
            lines.append(f"  CONFUSION: {', '.join(parts)}")

        bench_det = self._det_count.get("bench", 0)
        bench_loc = self._loc_count.get("bench", 0)
        if bench_det > 0 and bench_loc == 0:
            lines.append("  WARNING bench detected but NEVER localized (LiDAR association fail?)")
        elif bench_det == 0:
            lines.append("  WARNING bench NEVER detected by YOLO so far")

        self.get_logger().info("\n".join(lines))

        if self._csv_file:
            self._csv_file.flush()

    # -- Helpers --

    @staticmethod
    def _trim(timestamps, window=10.0):
        cutoff = time.monotonic() - window
        while timestamps and timestamps[0] < cutoff:
            timestamps.pop(0)

    @staticmethod
    def _fps(timestamps):
        if len(timestamps) < 2:
            return 0.0
        span = timestamps[-1] - timestamps[0]
        return (len(timestamps) - 1) / span if span > 0 else 0.0

    @staticmethod
    def _avg_interval(timestamps):
        if len(timestamps) < 2:
            return 0.0
        intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
        return sum(intervals) / len(intervals)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticRuntimeDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
