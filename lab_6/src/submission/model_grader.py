#!/usr/bin/env python3
import os
import sys
import argparse
import csv
import cv2
import numpy as np

# ------------------------------------------------------------------------------
#                  DO NOT MODIFY FUNCTION NAMES OR ARGUMENTS
# ------------------------------------------------------------------------------

def initialize_model(model_path=None):
    """
    Initialize and return your trained model.
    You MUST modify this function to load and/or construct your model.
    DO NOT change the function name or its input/output.
    
    Args:
        model_path: The path to your pretrained model file (if one is needed).
    Returns:
        model: Your trained model.
    """

    # YOLOv8m-cls exported to ONNX, run on CPU with OpenCV's DNN module (no torch needed).
    # Default: model.onnx next to this script.
    if model_path is None:
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.onnx")

    model = cv2.dnn.readNetFromONNX(model_path)

    return model

def predict(model, image):
    """
    Run inference on a single image using your model.
    You MUST modify this function to perform prediction.
    DO NOT change the function signature.
    
    Args:
        model: The model object returned by initialize_model().
        image: The input image (as a NumPy array) to classify.
    
    Returns:
        int: The predicted class label.
    """

    # Must match training preprocessing exactly (supplemental/preprocess.py):
    # squash-resize the whole BGR frame to 224x224 (INTER_AREA), BGR->RGB, scale to [0, 1], NCHW.
    imgsz = 224
    x = cv2.resize(image, (imgsz, imgsz), interpolation=cv2.INTER_AREA)
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    x = np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    model.setInput(x)
    probs = np.asarray(model.forward()).reshape(-1)  # softmax over 6 classes
    prediction = int(np.argmax(probs))

    return prediction

# ------------------------------------------------------------------------------
#                      DO NOT MODIFY ANY CODE BELOW THIS LINE
# ------------------------------------------------------------------------------

def load_validation_data(data_path):
    """
    Load validation images and labels from the given directory.
    Expects a 'labels.txt' file in the directory and images in .png format.
    
    Args:
        data_path (str): Path to the validation dataset.
    
    Returns:
        list of tuples: Each tuple contains (image_path, true_label)
    """
    labels_file = os.path.join(data_path, "labels.txt")
    data = []
    with open(labels_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Assumes row[0] is the image filename (without extension) and row[1] is the label.
            image_file = os.path.join(data_path, row[0] + ".png")  # Modify if images use a different extension.
            data.append((image_file, int(row[1])))
    return data

def evaluate_model(model, validation_data):
    """
    Evaluate the model on the validation dataset.
    Computes and prints the confusion matrix and overall accuracy.
    
    Args:
        model: The model object.
        validation_data (list): List of tuples (image_path, true_label).
    """
    num_classes = 6  # Number of classes (adjust if needed)
    confusion_matrix = np.zeros((num_classes, num_classes), dtype=np.int32)
    correct = 0
    total = len(validation_data)
    
    for image_path, true_label in validation_data:
        # Read the image
        image = cv2.imread(image_path)
        if image is None:
            print("Warning: Could not load image:", image_path)
            continue
        # Get the predicted label using the student's implementation.
        predicted_label = predict(model, image)
        
        if predicted_label == true_label:
            correct += 1
        confusion_matrix[true_label][predicted_label] += 1
        print(f"Image: {os.path.basename(image_path)} - True: {true_label}, Predicted: {predicted_label}")
    
    accuracy = correct / total if total > 0 else 0
    print("\nTotal accuracy:", accuracy)
    print("Confusion Matrix:")
    print(confusion_matrix)

def main():
    parser = argparse.ArgumentParser(description="Model Grader for Lab 6")
    parser.add_argument("--data_path", type=str, required=True,
                        help="Path to the validation dataset directory (must contain labels.txt and images)")
    parser.add_argument("--model_path", type=str, required=False,
                        help="Path to the trained model file (if applicable)")
    args = parser.parse_args()
    
    # Path to the validation dataset directory from command line argument.
    VALIDATION_DATASET_PATH = args.data_path

    # Path to the trained model file from command line argument.
    MODEL_PATH = args.model_path
    
    # Load validation data.
    validation_data = load_validation_data(VALIDATION_DATASET_PATH)
    
    # Initialize the model using the student's implementation.
    model = initialize_model(MODEL_PATH) if MODEL_PATH else initialize_model()
    
    # Evaluate the model on the validation dataset.
    evaluate_model(model, validation_data)

if __name__ == "__main__":
    main()
