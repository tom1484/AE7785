# AE7785 Lab 6: Sign Classification

YOLOv8-cls trained with `ultralytics` on the Mac (MPS/CPU), exported to ONNX, and run at grading
time with **OpenCV DNN only** (`opencv-python` + `numpy`, no torch).

## Layout

```
configs/            experiment configs (base.yaml + experiments/*.yaml that inherit it)
data/
  archives/         original course zips (tracked)
  raw/              extracted image folders, each with labels.txt (not tracked)
  splits/           generated train/val ImageFolder trees (not tracked)
provided/           course handout + original files, kept unmodified for reference
src/lab6/           library code (data loading, preprocessing, training, export, eval)
scripts/            command-line entry points
outputs/
  runs/             ultralytics training runs
  models/           exported .onnx models
  reports/          accuracy / confusion / latency tables
submission/         assembled Lab 6 submission (built by a script; not tracked)
```

## Setup

```
uv sync                      # creates .venv with training + dev deps
source .venv/bin/activate
```

Restore raw data from the archives if `data/raw/` is missing:

```
for z in data/archives/*.zip; do unzip -q "$z" -d data/raw/; done
```

## Plan

Train yolov8m-cls -> s -> n with identical settings; evaluate each through the
ONNX + cv2.dnn path; shrink until accuracy drops more than 1 point below `m` or
left/right confusions appear.

## Traps

- `fliplr` must be 0 (mirroring swaps left/right arrows).
- `predict()` must reproduce training preprocessing exactly (BGR->RGB, resize, /255).
- `provided/generate_requirements.py` scans root-level .py files of the *submission* dir.
