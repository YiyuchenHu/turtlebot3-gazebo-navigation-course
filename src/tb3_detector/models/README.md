# tb3_detector/models

Place YOLOv8 weight files here.

## Required for the assignment

| Filename       | Download |
|----------------|----------|
| `yolov8n.pt`   | `wget -O src/tb3_detector/models/yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt` |

Rebuild (`colcon build --packages-select tb3_detector`) after downloading so
the weights are copied into the install tree, or point the `model_path`
parameter at an absolute path.

## Naming convention

- `yolov8n.pt`           — official nano weights (COCO-80) — used by the assignment
- `yolov8s.pt`           — official small weights (COCO-80), optional upgrade
- `yolov8n_tb3_lab.pt`   — custom fine-tuned weights (future work)

## .gitignore

Weight files (`.pt`) are excluded from version control (large binary files).
Use git-lfs or a separate model registry if you need to share weights with a team.
