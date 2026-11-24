# lab_final_vision

YOLO sign detector for the AE7785 final. Trained here with ultralytics, exported to ONNX,
run on the ROS side with OpenCV DNN (`team85_final/vision/detector.py`).
Plan and status: `.ai/PROJECT.md`.

    uv sync                  # training deps
    uv sync --group label    # + zero-shot models for bootstrapping boxes
