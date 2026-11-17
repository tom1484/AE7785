AE7785 Lab 6 - Sign classifier
================================

Model
-----
YOLOv8m-cls (ultralytics, ImageNet-pretrained) fine-tuned on the provided sign images with
robustness augmentation, exported to ONNX (model.onnx). Grading runs the ONNX model on CPU with
OpenCV's DNN module, so model_grader.py itself only needs opencv-python and numpy (no torch, no GPU).

Grading
-------
From this directory:

    python3 model_grader.py --data_path ./val_dataset --model_path ./model.onnx

--model_path may be omitted; model.onnx next to model_grader.py is loaded by default.
Only the designated parts of model_grader.py were changed (initialize_model and predict); no extra
imports were needed. Tested with opencv-python 4.11 and 5.0.

Training (optional, reproduces model.onnx)
------------------------------------------
    pip install -r requirements.txt
    python3 train_model.py --data_dirs ./2025F_imgs ./2025F_Gimgs ./2026F_imgs --out ./model.onnx

Uses CUDA or Apple MPS if available, otherwise CPU. Pretrained weights are downloaded by ultralytics
on first use. Intermediate files go to ./work. requirements.txt lists ultralytics/torch because of
this training script; they are not needed to run model_grader.py.

Files
-----
model_grader.py           grader with initialize_model() / predict() implemented
model.onnx                trained model
train_model.py            training + ONNX export
supplemental/preprocess.py  resize used for training (mirrored inline in predict())
supplemental/corrupt.py     train-time corruption augmentation
requirements.txt          generated with generate_requirements.py

Notes
-----
- Preprocessing: the whole frame is resized (no crop) to 224x224 with INTER_AREA, BGR->RGB, /255.
- Horizontal flip augmentation is disabled on purpose: a mirrored left arrow is a right arrow.
- Validation during development: trained on the 2025 folders only, the same recipe scored 100% on
  all 153 images of 2026F_imgs, and 99.7% on 765 synthetically corrupted copies of them.
