signs.onnx: YOLO11n sign detector, ONNX opset 12, input 1x3x416x416, output (1, 9, 3549)
= cx, cy, w, h + 5 class scores, no NMS in the graph. Classes in order:
left, right, do_not_enter, stop, goal (config/final.yaml class_names, imgsz 416).

Built in ~/Workspace/Courses/AE7785/lab_final_vision (commit 345796e):
run outputs/runs/det/yolo11n_all_416 (all 4 lab_6 folders, 591 images / 627 boxes,
80 epochs, last.pt). Held-out estimate (same recipe, 2026F_imgs held out): P 0.938 R 0.923,
0 left<->right swaps, front sign 133/133; weak on far stop vs goal (see that repo's notes).
