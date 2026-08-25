#!/usr/bin/env python3
"""
detector_core.py  —  Stage-1 perception: YOLOv8 inference wrapper.

████████████████████████████████████████████████████████████████████████████
██                                                                        ██
██   STUDENT ASSIGNMENT — THIS FILE IS THE CORE OF YOUR TASK              ██
██                                                                        ██
██   The rest of the navigation stack (localizer, memory, query, Nav2)    ██
██   is provided and working. It is waiting for real detections from      ██
██   this class. Until you implement load() and infer(), the robot        ██
██   explores and maps fine, but every "go to person" command answers     ██
██   "no objects in memory".                                              ██
██                                                                        ██
██   Read INSTRUCTIONS.md at the repository root before you start.        ██
██                                                                        ██
████████████████████████████████████████████████████████████████████████████

Responsibilities of this class:
  - Load a YOLOv8 model from a configurable local path.
  - Run inference on a BGR numpy image (from cv_bridge).
  - Return a list of Detection dicts (format below).

NOT responsible for:
  - Coordinate projection to 3D (→ tb3_localizer, provided)
  - Maintaining object history  (→ tb3_memory,    provided)
  - Answering semantic queries  (→ tb3_query,     provided)
  - Sending Nav2 goals          (→ tb3_nav_adapter / tb3_coordinator, provided)

============================================================================
OUTPUT CONTRACT — do not change key names; the provided localizer and the
node wrapper (detector_node.py) rely on them:

    {
        "label":      str,          # COCO class name, e.g. "person", "bench"
        "conf":       float,        # confidence, 0.0 – 1.0
        "bbox_xyxy":  [x1, y1, x2, y2],   # pixels (float), see convention below
        "track_id":   int | None,   # None unless you enable tracking
    }

Pixel-coordinate convention (standard image coordinates):
    - origin (0, 0) is the TOP-LEFT corner of the image
    - x grows to the RIGHT, y grows DOWN
    - (x1, y1) = top-left corner of the box, (x2, y2) = bottom-right corner
    - values are in pixels of the ORIGINAL /camera/image_raw resolution —
      if you resize the image for inference, scale the boxes back!
      (ultralytics already returns boxes in original-image pixels.)
    The provided localizer uses the bbox centre x = (x1+x2)/2 to compute a
    bearing through the camera FOV, so a wrong x coordinate sends the robot
    in a wrong direction.
============================================================================
"""

from __future__ import annotations
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# TODO(student) ── Step 0: make ultralytics importable
# ---------------------------------------------------------------------------
# Install the Python dependencies (they are NOT apt packages):
#
#     pip install ultralytics
#     # CPU-only torch (smaller download, sufficient for this course):
#     pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
#
# Then uncomment an import here, guarded so the package still builds when
# ultralytics is missing (see the original pattern below):
#
#     try:
#         from ultralytics import YOLO as _UltralyticsYOLO
#         _ULTRALYTICS_AVAILABLE = True
#     except ImportError:
#         _UltralyticsYOLO = None
#         _ULTRALYTICS_AVAILABLE = False
# ---------------------------------------------------------------------------


# Keys every downstream consumer may rely on:
DETECTION_KEYS = ("label", "conf", "bbox_xyxy", "track_id")


class DetectorCore:
    """
    Thin wrapper around a YOLOv8 model.  ***Currently a stub.***

    Usage (once implemented)::

        core = DetectorCore(model_path="models/yolov8n.pt", conf_threshold=0.12)
        core.load()                          # loads weights once at startup
        detections = core.infer(bgr_image)   # list of dicts (see module docstring)

    Parameters
    ----------
    model_path : str | Path
        Absolute or relative path to the YOLOv8 .pt weights file.
    conf_threshold : float
        Minimum confidence to include a detection (0.0 – 1.0).
        The shipped config uses 0.12 — unusually low, but validated for this
        Gazebo scene where the marble table is a borderline "bench". Typical
        real-world values are 0.25 – 0.5. Tune in config/detector.yaml, not here.
    class_filter : list[str] | None
        If given, only return detections whose label is in this list.
        None means return all detected classes.
        IMPORTANT: entries are COCO *detector labels* ("bench", "stop sign",
        "person"), NOT the task-level semantic names — see the mapping note below.
    device : str
        Torch device string, e.g. "cpu", "cuda:0".
    enable_tracking : bool
        If True, use model.track() instead of model.predict() (ByteTrack).
        Optional — the course task only requires plain per-frame detection.

    ------------------------------------------------------------------------
    COCO label → task label mapping (important!)
    ------------------------------------------------------------------------
    YOLOv8's COCO-80 class names do not always match what this project calls
    an object. The mapping used by the rest of the stack lives in
    src/tb3_frontier_exploration/config/semantic_targets.yaml:

        task name    COCO detector_label      Gazebo model
        ---------    -------------------      ------------
        person   ←   "person"                 person_standing
        table    ←   "bench"     (class 13)   table_marble
        stop_sign ←  "stop sign" (class 12,   stop_sign
                      note the space!)

    In this simulation YOLO sees the marble table as a *bench* — do NOT
    expect "dining table" (COCO class 60); it does not fire reliably here.
    Your DetectorCore must simply report raw COCO labels; the provided
    downstream nodes translate them using semantic_targets.yaml.
    """

    def __init__(
        self,
        model_path: str | Path,
        conf_threshold: float = 0.35,
        class_filter: list[str] | None = None,
        device: str = "cpu",
        enable_tracking: bool = False,
    ) -> None:
        self.model_path = Path(model_path)
        self.conf_threshold = float(conf_threshold)
        self.class_filter = set(class_filter) if class_filter else None
        self.device = device
        self.enable_tracking = enable_tracking

        self._model: Any = None   # set by load()

    # ------------------------------------------------------------------
    def load(self) -> None:
        """
        Load model weights.  Called once by detector_node at startup.

        ████ TODO(student) ── Step 1: download weights, Step 2: load them ████

        1. Download the pretrained YOLOv8-nano weights (~6 MB, COCO-80):

               wget -O src/tb3_detector/models/yolov8n.pt \
                 https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt

           The models/ folder git-ignores *.pt on purpose — weights stay
           local. Rebuild (colcon build --packages-select tb3_detector)
           after downloading so the file is copied into the install tree,
           or pass an absolute path via model_path.

        2. Implement loading here:
             - raise RuntimeError with a helpful message if ultralytics is
               not installed (keep startup errors readable for the grader);
             - raise FileNotFoundError if self.model_path is missing;
             - otherwise create the model:  self._model = YOLO(str(self.model_path))
               and move it to self.device.

        The stub below intentionally does NOT raise, so that the unmodified
        course repo starts up and publishes empty detections.
        """
        # TODO(student): replace this stub with real model loading.
        logger.warning(
            "DetectorCore.load(): STUB — no model loaded. "
            "detector_node will publish EMPTY detections until you implement "
            "DetectorCore (see INSTRUCTIONS.md)."
        )
        self._model = None

    # ------------------------------------------------------------------
    def infer(self, bgr_image) -> list[dict]:
        """
        Run inference on a single BGR uint8 numpy array (shape H×W×3).

        Returns a (possibly empty) list of detection dicts, format specified
        in the module docstring.

        ████ TODO(student) ── Step 3: implement inference ████

        Outline (ultralytics does most of the work):

            results = self._model.predict(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )
            for result in results:
                for each box in result.boxes:
                    label = result.names[int(box.cls)]     # COCO name, e.g. "person"
                    ... skip it if self.class_filter is set and label not in it ...
                    conf  = float(box.conf)
                    xyxy  = box.xyxy[0].tolist()           # [x1, y1, x2, y2] pixels
                    ... append {"label": label, "conf": conf,
                                "bbox_xyxy": xyxy, "track_id": None} ...

        Notes:
          - ultralytics accepts BGR numpy arrays directly; no colour
            conversion or resizing is needed (boxes come back in original
            image pixels).
          - Return [] when nothing is detected — never None.
          - Optional: if self.enable_tracking, use self._model.track(...,
            persist=True) and fill "track_id" from box.id.
        """
        # TODO(student): replace this stub with real YOLO inference.
        return []

    # ------------------------------------------------------------------
    @property
    def is_loaded(self) -> bool:
        return self._model is not None
