# Dataset — GO2 Seeing-Eye Dog

Documents the dataset collection protocol, annotation approach, and class breakdown
for the custom YOLOv8 perception model used in this project.

## Classes

| ID | Class | Description | Role |
|----|-------|-------------|------|
| 0 | `owner_person` | The target user to follow/guide | Primary detection target |
| 1 | `wrist_marker` | AprilTag36h11 / ArUco marker on wrist | Identity confirmation cue |
| 2 | `phone_marker` | User's phone/tablet held in hand | Secondary identity cue |
| 3 | `follow_marker` | Colored vest / paddle / reflective hat | Visual anchor for following |

## Collection Protocol

### Hardware
- Intel RealSense D435i (RGB + depth, 640×480 @ 30 Hz)
- NVIDIA Jetson Orin (onboard compute)
- Unitree GO2 quadruped

### Environments
- Indoor corridors (controlled lighting)
- Indoor open spaces (variable lighting, crowd simulation)
- Outdoor walkways (natural lighting, partial occlusion from vegetation)

### Collection Procedure

1. Mount RealSense on GO2 at standard operating height (~0.5 m from floor)
2. Record ROS 2 bag files (`ros2 bag record /go2/camera/image_raw /go2/camera/depth/image_raw`)
3. Extract frames at 1 Hz using `training/collect_frames.py`
4. Filter for diversity (lighting, distance, pose, occlusion level)

```bash
# Extract frames from a bag file
python3 training/collect_frames.py \
    --bag /path/to/bag \
    --output ~/datasets/go2_perception/raw/ \
    --rate 1.0
```

### Annotation
- Tool: [Label Studio](https://labelstud.io/) with YOLO export format
- Format: YOLO v8 (normalized xywh bounding boxes per class)
- Each annotator labels all visible instances of all 4 classes per frame
- Ambiguous/occluded instances (>70% occluded) are skipped

### Augmentation
Offline augmentation via `training/augment_dataset.py` targets underrepresented classes:

```bash
python3 training/augment_dataset.py \
    --dataset ~/datasets/go2_perception \
    --target-count 800 \
    --classes 1 2 3   # wrist_marker, phone_marker, follow_marker
```

Augmentations applied: horizontal flip, brightness/contrast jitter (±30%),
Gaussian blur (σ ≤ 1.5), random crop (±10%), hue/saturation shift.

## Dataset Status

> **Current status: collection in progress.**
> Update this table after each collection session.

| Split | `owner_person` | `wrist_marker` | `phone_marker` | `follow_marker` | Total frames |
|-------|---------------|---------------|---------------|----------------|-------------|
| Train | — | — | — | — | — |
| Val   | — | — | — | — | — |
| Test  | — | — | — | — | — |

## Directory Layout

```
~/datasets/go2_perception/
├── dataset.yaml          # YOLO dataset config (path, nc, names)
├── train/
│   ├── images/           # .jpg frames
│   └── labels/           # .txt YOLO annotations
├── val/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
```

## Training

See `training/train.sh` for the full training pipeline (YOLOv8s, AdamW, 150 epochs,
offline augmentation, early stopping). Results go to `~/models/go2_perception/`.
