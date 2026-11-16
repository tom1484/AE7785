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
- Corruptions are eval-only so far (not in training).
- Next (pending user choice): train-time corruption aug (split train/held-out corruption sets),
  s@160 run; then submission assembler.

## Known traps
- venv has opencv 5.0.0; the TA may use 4.x. Verify cv2.dnn ONNX loading on both, or pin.
- `fliplr` must be 0.
- `provided/` files stay unmodified; the working grader copy goes in `submission/`.
