import supervision as sv
import numpy as np
from roboflow import Roboflow
import pandas as pd
import os
import cv2

# Initialize Roboflow
rf = Roboflow(api_key="hFyT16eNtzwRsmbWtu97")
project = rf.workspace("gptnlplabassistant4").project("components-model-v2tgx")

# FIXME: change version number as necessary
# Load your trained model
model = project.version("6").model

# FIXME: change filepath for version as necessary
# Load your test/validation dataset
dataset = sv.DetectionDataset.from_yolo(
    images_directory_path="/home/raquel/Documents/Components Model.v5-2026-02-20-5-41pm-yolov11-no-inductor.yolov11/test/images",
    annotations_directory_path="/home/raquel/Documents/Components Model.v5-2026-02-20-5-41pm-yolov11-no-inductor.yolov11/test/labels",
    data_yaml_path="/home/raquel/Documents/Components Model.v5-2026-02-20-5-41pm-yolov11-no-inductor.yolov11/data.yaml"
)

# Define callback function for Roboflow inference
def callback(image: np.ndarray) -> sv.Detections:
    """
    Run inference on image using Roboflow model and return detections
    """
    # Roboflow inference returns predictions in JSON format
    result = model.predict(image, confidence=40, overlap=30).json()

    # Convert Roboflow predictions to supervision Detections format
    detections = []

    if 'predictions' in result:
        xyxy = []
        class_ids = []
        confidences = []

        for pred in result['predictions']:
            # Convert from center format to xyxy format
            x_center = pred['x']
            y_center = pred['y']
            width = pred['width']
            height = pred['height']

            x1 = x_center - width / 2
            y1 = y_center - height / 2
            x2 = x_center + width / 2
            y2 = y_center + height / 2

            xyxy.append([x1, y1, x2, y2])

            # Get class_id from class name
            class_name = pred['class']
            class_id = dataset.classes.index(class_name) if class_name in dataset.classes else 0
            class_ids.append(class_id)

            confidences.append(pred['confidence'])

        if xyxy:
            detections = sv.Detections(
                xyxy=np.array(xyxy),
                class_id=np.array(class_ids),
                confidence=np.array(confidences)
            )
        else:
            # Return empty detections if no predictions
            detections = sv.Detections.empty()
    else:
        detections = sv.Detections.empty()

    return detections


# Calculate confusion matrix
print("Computing confusion matrix... This may take a while depending on dataset size.")
confusion_matrix = sv.ConfusionMatrix.benchmark(
    dataset=dataset,
    callback=callback
)

# Print results
print("\nConfusion Matrix:")
print(confusion_matrix.matrix)

print("\nClasses:")
print(confusion_matrix.classes)

# Calculate accuracy and other metrics
total_predictions = confusion_matrix.matrix.sum()
correct_predictions = np.trace(confusion_matrix.matrix)  # Sum of diagonal
accuracy = correct_predictions / total_predictions if total_predictions > 0 else 0

print(f"\nOverall Accuracy: {accuracy:.2%}")

# Per-class metrics as table
metrics_data = []
for i, class_name in enumerate(confusion_matrix.classes):
    if i < len(confusion_matrix.matrix):
        tp = confusion_matrix.matrix[i, i]
        fp = confusion_matrix.matrix[:, i].sum() - tp
        fn = confusion_matrix.matrix[i, :].sum() - tp

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

        metrics_data.append({
            'Class': class_name,
            'Precision': f"{precision:.2%}",
            'Recall': f"{recall:.2%}",
            'F1-Score': f"{f1:.2%}",
            'TP': int(tp),
            'FP': int(fp),
            'FN': int(fn)
        })

metrics_df = pd.DataFrame(metrics_data)
print("\nPer-Class Metrics:")
print(metrics_df.to_string(index=False))

# FIXME: EDIT BELOW 2 LINES TO CHANGE OUTPUT FILE NAME
# Save metrics to CSV
metrics_df.to_csv('per_class_metrics_v6.csv', index=False)
print("\nMetrics saved to 'per_class_metrics_v6.csv'")

# Optional: Visualize the confusion matrix
try:
    import matplotlib.pyplot as plt
    import seaborn as sns

    plt.figure(figsize=(10, 8))
    sns.heatmap(confusion_matrix.matrix,
                annot=True,
                fmt='g',
                cmap='Blues',
                xticklabels=confusion_matrix.classes,
                yticklabels=confusion_matrix.classes)
    plt.xlabel('Predicted')
    plt.ylabel('Actual')
    plt.title('Confusion Matrix')
    plt.tight_layout()

    # FIXME: EDIT BELOW 2 LINES TO CHANGE OUTPUT FILE NAME
    plt.savefig('confusion_matrix_v6.png', dpi=300, bbox_inches='tight')
    print("\nConfusion matrix visualization saved as 'confusion_matrix_v6.png'")
except ImportError:
    print("\nInstall matplotlib and seaborn to visualize: pip install matplotlib seaborn")

# Export annotated images with prediction bounding boxes and confidence values
# FIXME: Change output directory as needed
output_dir = "annotated_predictions_v6"
os.makedirs(output_dir, exist_ok=True)

print(f"\nExporting annotated images to '{output_dir}/'...")

# Create annotators
box_annotator = sv.BoxAnnotator(thickness=2)
label_annotator = sv.LabelAnnotator(text_scale=0.5, text_thickness=1)

# Iterate through all images in the dataset
image_count = 0
for image_path, image, _ in dataset:
    image_name = os.path.basename(image_path)

    # Run inference on the image
    detections = callback(image)

    # Create labels with class name and confidence
    labels = []
    if len(detections) > 0 and detections.confidence is not None:
        for class_id, confidence in zip(detections.class_id, detections.confidence):
            class_name = dataset.classes[class_id] if class_id < len(dataset.classes) else "unknown"
            labels.append(f"{class_name}: {confidence:.2f}")

    # Annotate the image
    annotated_image = image.copy()
    annotated_image = box_annotator.annotate(scene=annotated_image, detections=detections)
    annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)

    # Save the annotated image
    output_path = os.path.join(output_dir, f"pred_{image_name}")
    cv2.imwrite(output_path, cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
    image_count += 1

print(f"Exported {image_count} annotated images to '{output_dir}/'")
print("Each image shows prediction bounding boxes with class labels and confidence scores.")