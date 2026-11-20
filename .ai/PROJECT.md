# Project entrypoint (agent-neutral)

AE7785 final (demo 2026-12-07): train a YOLO **detection** model for the maze signs, for
online inference on the TurtleBot pipeline. Consumer: ROS package
`~/Workspace/turtlebot3_ws/src/team85_final` (`team85_final/vision/detector.py`,
params in `config/final.yaml`, model at `models/signs.onnx`). Read its `CLAUDE.md` for the
whole-system plan. Predecessor: `../lab_6` (YOLOv8-cls, 100% on 4 folders; see its .ai/PROJECT.md).

## Why detection, not cls
Box position -> bearing of each sign -> which wall face it is on (graph planner uses signs
seen in passing), and the "sign in front" can be picked by box size/centeredness.

## Contract with team85_final (do not break silently)
- Classes, in this order: `[left, right, do_not_enter, stop, goal]` (no "empty" class;
  empty-wall images are background images with no boxes). Names map to
  `SignClass.from_name` in team85_final/vision/signs.py.
- Export: ultralytics detect ONNX, output (1, 4+nc, N) = cx,cy,w,h + class scores, no NMS
  in-graph. Runtime = OpenCV DNN 4.13 (humble conda env; no onnxruntime there).
- Preprocess in detector.py: BGR->RGB, letterbox to square imgsz with pad 114, /255.
  Train/export with the same imgsz as `final.yaml: sign_detector.imgsz` (draft value 320).
- `fliplr: 0` (mirroring swaps left/right) -- same trap as lab 6.

## Plan
1. Data: start from ../lab_6/data/raw (4 folders, ~595 imgs, 410x308, class label per
   image, exactly one sign per non-empty image). Convert to YOLO format with boxes.
2. Boxes: bootstrap automatically, then review every box on contact sheets and fix by hand.
   - Naive HSV saturation boxing was tried (2026-11-19) and FAILS: pale-green arrows are
     low saturation, orange walls saturate like the red signs. 2025 sets also have blue
     arrows and straight arrows.
   - Next to try: zero-shot detector (OWLv2 or Grounding DINO via transformers), keep the
     top-1 box per image (one sign per image is guaranteed); fallback: segment pixels far
     from the dominant wall color in Lab space.
3. Add robot-view data: frames from the TurtleBot camera in the final maze at several
   distances/angles, incl. multiple signs per frame and dim corners (lab says lighting is
   poor near walls). Gazebo sign textures differ (blue/green arrows, orange stop/goal);
   capture sim frames too if the sim is used for end-to-end tests.
4. Train yolov8n / yolo11n detect at imgsz 320 (then try 256/416), corruption aug as in
   lab_6 (lfv can reuse ../lab_6/src/lab6/corrupt.py). Train on MPS.
5. Eval: mAP50, plus a classification-style check on lab_6 images (top box class == label,
   empty -> no box) and left<->right confusions (worst error for navigation).
6. Export ONNX, check parity ultralytics vs cv2.dnn 4.13, measure CPU latency, copy to
   team85_final/models/signs.onnx and update final.yaml.

## Current state (2026-11-19)
- Scaffold only: pyproject (not yet `uv sync`ed), dirs, this plan. No data converted yet.

## Known traps
- Don't copy lab_6 data into git; reference ../lab_6/data/raw or archive zips under data/archives.
