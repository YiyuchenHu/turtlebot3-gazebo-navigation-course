#!/usr/bin/env python3
"""
detector_core.py  —  Stage-1 perception: YOLOv8 inference wrapper.

████████████████████████████████████████████████████████████████████████████
██                                                                        ██
██   REFERENCE IMPLEMENTATION — this is the completed assignment.         ██
██                                                                        ██
██   On the `main` branch this file ships as a stub with TODOs; that is   ██
██   the version students receive. Here load() and infer() are filled in  ██
██   so the whole semantic-navigation stack runs end to end.              ██
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
# Optional import — graceful failure if ultralytics is not installed.
# Keeping this guarded means the package still *builds* without the Python
# dependencies; load() is where the missing dependency becomes a hard error.
#
#     pip install 'ultralytics==8.4.31'   # pinned; yolo26n needs >= 8.4.x
#     pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
# ---------------------------------------------------------------------------
try:
    from ultralytics import YOLO as _UltralyticsYOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _UltralyticsYOLO = None
    _ULTRALYTICS_AVAILABLE = False
    logger.warning(
        "ultralytics not found. Install with:  pip install 'ultralytics==8.4.31'\n"
        "detector_core will raise RuntimeError on load() until then."
    )


# Keys every downstream consumer may rely on:
DETECTION_KEYS = ("label", "conf", "bbox_xyxy", "track_id")


class DetectorCore:
    """
    Thin wrapper around a YOLOv8 model.

    Usage::

        core = DetectorCore(model_path="models/yolo26n.pt", conf_threshold=0.35)
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
    DetectorCore simply reports raw COCO labels; the provided downstream
    nodes translate them using semantic_targets.yaml.
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

        Raises RuntimeError if ultralytics is missing, FileNotFoundError if
        the weights file is absent — both with a message the grader can read.

        Weights are git-ignored on purpose; download them with:

            wget -O src/tb3_detector/models/yolo26n.pt \
              https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt

        then rebuild (colcon build --packages-select tb3_detector) so the file
        is copied into the install tree, or pass an absolute model_path.
        """
        if not _ULTRALYTICS_AVAILABLE:
            raise RuntimeError(
                "ultralytics package is not installed. "
                "Run:  pip install 'ultralytics==8.4.31'"
            )
        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Model file not found: {self.model_path}\n"
                "► See INSTRUCTIONS.md → model download step."
            )

        logger.info("Loading YOLOv8 model from %s on device=%s", self.model_path, self.device)
        self._model = _UltralyticsYOLO(str(self.model_path))
        self._model.to(self.device)
        logger.info("Model loaded. Classes: %s", list(self._model.names.values()))

    # ------------------------------------------------------------------
    def infer(self, bgr_image) -> list[dict]:
        """
        Run inference on a single BGR uint8 numpy array (shape H×W×3).

        Returns a (possibly empty) list of detection dicts, format specified
        in the module docstring. Never returns None.

        ultralytics accepts BGR numpy arrays directly; no colour conversion or
        resizing is needed, and boxes come back in original-image pixels.
        """
        if self._model is None:
            raise RuntimeError("DetectorCore.load() has not been called yet.")

        if self.enable_tracking:
            results = self._model.track(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                persist=True,
                verbose=False,
            )
        else:
            results = self._model.predict(
                bgr_image,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
            )

        detections: list[dict] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            names = result.names  # int -> str

            for i in range(len(boxes)):
                label = names[int(boxes.cls[i].item())]
                if self.class_filter and label not in self.class_filter:
                    continue

                conf = float(boxes.conf[i].item())
                xyxy = boxes.xyxy[i].tolist()  # [x1, y1, x2, y2]

                track_id = None
                if self.enable_tracking and boxes.id is not None:
                    track_id = int(boxes.id[i].item())

                detections.append(
                    {
                        "label":     label,
                        "conf":      conf,
                        "bbox_xyxy": xyxy,
                        "track_id":  track_id,
                    }
                )

        return detections

    # ------------------------------------------------------------------
    @property
    def is_loaded(self) -> bool:
        return self._model is not None
