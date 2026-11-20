# Project entrypoint (agent-neutral)

AE7785 final (demo 2026-12-07): train a YOLO **detection** model for the maze signs, for
online inference on the TurtleBot pipeline. Consumer: ROS package
`~/Workspace/turtlebot3_ws/src/team85_final` (`team85_final/vision/detector.py`,
params in `config/final.yaml`, model at `models/signs.onnx`). Read `~/Workspace/turtlebot3_ws/CLAUDE.md` for the
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
- Scaffold + `uv sync --group label` done (torch 2.15 MPS, ultralytics 8.4.215,
  transformers 5.23, cv2 5.1; CLIP from git for YOLOE/YOLO-World).
- Zero-shot benchmark (`scripts/bench_zeroshot.py`, 30 lab_6 images = 6/class over all
  folders, prompts arrow / stop sign / star / u-turn arrow sign, top-1 box, M2 MPS warm):

  | model | ms/img | found | quality (contact sheets in outputs/review/) |
  |---|---|---|---|
  | Grounding DINO tiny (IDEA-Research/grounding-dino-tiny) | 1121 | 30/30 | 29 tight; 1 whole-wall box (do-not-enter) |
  | OWLv2 base (google/owlv2-base-patch16-ensemble) | 802 | 30/30 | 28 tight; 2 whole-wall (do-not-enter); scores ~0.6-0.9 |
  | YOLOE-11s-seg | 50 | 28/30 | tight when found, but scores 0.01-0.1; missed blue curved arrow, dim star |
  | YOLO-World v2 s | 33 | 13/30 | unusable |

  Decision: label on the laptop (OWLv2 + GDINO on ~595 imgs is ~20 min). No remote
  (Mac mini / dev-infra) image needed. Docker on macOS has no MPS anyway, so remote
  training in a container would be CPU-only and slower than the laptop.
- Labeling pipeline (2026-11-19):
  - `scripts/autolabel.py`: OWLv2 + GDINO top-3 boxes per lab_6 image -> `data/labels/auto.json`
    (resumable; `--reanalyse` recomputes flags/proposals from stored preds in seconds).
    Proposal: mean of the two top-1 boxes if IoU >= 0.8, else OWLv2's (tighter), skipping
    boxes >= 50% of the image. Flags: no_box, disagree, huge_box, low_score, edge, sign_on_empty.
  - `scripts/review.py`: OpenCV GUI over ALL images, flagged/low-score first. Writes
    `data/labels/labels.json` (human truth; autolabel never touches it) on every accept.
    Logic is in the window-free `Reviewer` class, tested in tests/test_review.py.
  - Both json files are tracked in git (labels.json is manual work: commit it often).
- Next: user reviews all images; then export reviewed labels to YOLO format (class id =
  lab_6 id - 1) with the lab_6 holdout split, train, export, hand to team85_final.

## Known traps
- Run the venv with the humble conda env deactivated (`env -i HOME=$HOME PATH=/usr/bin:/bin
  .venv/bin/python ...` from agent shells): its libomp clashes with torch's (OMP Error #15).
- uv venv has no pip: ultralytics' auto-install of CLIP fails; CLIP is in the `label` group.
- transformers 5: OWLv2 post-processing is `post_process_grounded_object_detection`.
- Box failure mode of both zero-shot models: whole-wall box on do-not-enter signs.
- Don't copy lab_6 data into git; reference ../lab_6/data/raw or archive zips under data/archives.
