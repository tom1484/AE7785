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
- Next (pending user choice): shrink imgsz (224 -> 160 -> 128) and/or harder robustness eval;
  then a submission assembler (model_grader.py with predict via cv2.dnn, standalone train script).

## Known traps
- venv has opencv 5.0.0; the TA may use 4.x. Verify cv2.dnn ONNX loading on both, or pin.
- `fliplr` must be 0.
- `provided/` files stay unmodified; the working grader copy goes in `submission/`.
