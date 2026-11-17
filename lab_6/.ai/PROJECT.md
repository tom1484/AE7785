# Project entrypoint (agent-neutral)

AE7785 Lab 6 (due 2026-11-20 23:59): 6-class sign classifier graded by
`model_grader.py` accuracy on a withheld set. Final demo (2026-12-07) reuses the model on a TurtleBot.
Handout: `provided/lab6_final.pdf`. Read `README.md` for layout, setup, and traps.

## Decisions
- Model family: ultralytics YOLOv8-cls; sweep m -> s -> n, stop shrinking when held-out
  accuracy drops >1 pt below m or left/right confusions appear.
- Grading runs on CPU via ONNX + `cv2.dnn` (opencv-python + numpy only). onnxruntime is the fallback.
- Training on Apple M2 with `device=mps`.

## Current state (2026-11-14)
- Done: pipeline (scripts/train.py -> split, train, ONNX export, parity, CPU eval -> outputs/reports/),
  scripts/summarize.py. ONNX runs identically in cv2 5.0, cv2 4.11, and onnxruntime.
- Sweep m/s/n x {stratified 80/20, holdout 2026F (train on 2025 only)}: ALL 100% accuracy.
  Val is saturated; only confidence margins differ (n worst margin 0.30 on holdout vs ~0.6 for m/s).
  CPU latency cv2.dnn: m 71 ms, s 23 ms, n 7.6 ms. ONNX: 63 / 20 / 5.8 MB.
- Caveat: the val split also drives early stopping/best.pt, so scores are slightly optimistic.
- imgsz sweep (n, holdout-2026): 224/160 100%, 128 98.7% (left->right error), 96 95.4%, 64 92.2%.
  Smallest clean-acceptable: n@160.
- Robustness (scripts/robustness.py, 16 corruptions x 3 sev + 5 composite copies/img; reports in
  outputs/reports/robustness__*.md). Holdout-2026 composite: n@224 91.4, n@160 91.9, n@128 91.5,
  s@224 96.1, m@224 96.2. Under noise n fails the >1pt rule; s@224 is smallest acceptable.
  Weak spots: severe darkness (brightness sev3), jpeg q10, noise sigma20. n errors mostly "->empty";
  m has 11 left<->right composite errors (worst kind for navigation), s has 1.
- Train-time aug (user choice: ALL 16 corruptions, test with a different image seed):
  configs/experiments/*_aug.yaml, lab6/augtrain.py, 60 epochs, export last.pt.
  Holdout-2026, test seed 1 (outputs/reports/robustness__holdout-2026F_imgs_seed0__testseed1.md),
  composite acc / L<->R errors (of 765):
    no-aug: n@224 90.6/5, n@160 91.1/7, s@224 94.1/12, m@224 95.2/15
    aug:    n@224 98.8/0, n@160 97.1/3, s@224 99.1/2, s@160 96.1/1, m@224 99.7/0
  160px + aug hurts (s_aug@160 clean 97.4%, misses -> empty). Smallest passing the >1pt rule:
  n_aug@224 (clean 100%, 7.6 ms CPU, 5.8 MB). Caveat: test noise = training noise distribution.
- User choice: SUBMIT m_aug@224; keep n_aug@224 for on-robot inference in the final.
- Lab 6 submission DONE (2026-11-16): outputs/Chen_Zhu_Lab6.zip (52 MB), staged in submission/.
  Sources: src/submission/{model_grader.py,train_model.py,readme.txt}; build with
  `python scripts/make_submission.py --model submission/model.onnx --requirements --zip Chen_Zhu_Lab6.zip`.
  model.onnx = m_aug trained on ALL 3 folders by the submitted train_model.py (60 ep, last.pt).
  Final n model: outputs/models/final_yolov8n_cls_aug_all224.onnx.
  Verified: fresh py3.12 venv from requirements.txt (cv2 4.11) and torch-free venv (cv2 5.0) both
  run the grader; train_model.py runs from the extracted zip.
- Trap: pipreqs emitted opencv_python twice (4.11 + 5.0); make_submission.py dedupes (keeps first).
- Not done: user must upload the zip to Canvas. Next project phase: the final (ROS navigation).

## Known traps
- venv has opencv 5.0.0; the TA may use 4.x. Verify cv2.dnn ONNX loading on both, or pin.
- `fliplr` must be 0.
- `provided/` files stay unmodified; the working grader copy goes in `submission/`.
