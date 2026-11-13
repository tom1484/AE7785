# Project entrypoint (agent-neutral)

AE7785 Lab 6 (due 2026-11-20 23:59): 6-class sign classifier graded by
`model_grader.py` accuracy on a withheld set. Final demo (2026-12-07) reuses the model on a TurtleBot.
Handout: `provided/lab6_final.pdf`. Read `README.md` for layout, setup, and traps.

## Decisions
- Model family: ultralytics YOLOv8-cls; sweep m -> s -> n, stop shrinking when held-out
  accuracy drops >1 pt below m or left/right confusions appear.
- Grading runs on CPU via ONNX + `cv2.dnn` (opencv-python + numpy only). onnxruntime is the fallback.
- Training on Apple M2 with `device=mps`.

## Current state
- Done: directory layout, configs (base + m/s/n), uv venv (`uv sync`), deps verified, MPS works.
- Next: split builder (labels.txt -> ImageFolder in data/splits/), train script, ONNX export,
  cv2.dnn evaluator + PyTorch/ONNX parity check, submission assembler.

## Known traps
- venv has opencv 5.0.0; the TA may use 4.x. Verify cv2.dnn ONNX loading on both, or pin.
- `fliplr` must be 0.
- `provided/` files stay unmodified; the working grader copy goes in `submission/`.
