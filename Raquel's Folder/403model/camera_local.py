import sys

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import base64
import cv2
import requests
import threading
import time

# --- Local inference server configuration ---
# Roboflow inference server running in Docker on this device (port 9001, no auth)
INFERENCE_SERVER = "http://localhost:9001"
MODEL_ID = "components-model-v2tgx/5"   # no workspace prefix
CONFIDENCE = 0.40   # 0.0–1.0
IOU_THRESHOLD = 0.30

INFERENCE_URL = f"{INFERENCE_SERVER}/infer/object_detection"


def run_inference(frame):
    """POST a BGR frame to the local Roboflow inference server and return predictions list."""
    _, buf = cv2.imencode(".jpg", frame)
    b64 = base64.b64encode(buf).decode("utf-8")

    resp = requests.post(
        INFERENCE_URL,
        json={
            "id": "camera_local",
            "api_key": os.getenv("RoboFlow_KEY"),
            "model_id": MODEL_ID,
            "image": {"type": "base64", "value": b64},
            "confidence": CONFIDENCE,
            "iou_threshold": IOU_THRESHOLD,
        },
        timeout=10,
    )
    resp.raise_for_status()
    return resp.json().get("predictions", [])


# Open the ArduCam (USB camera)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera. Try a different index (1, 2, etc.).")
    exit(1)

# Set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Drawing settings
BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)
TEXT_BG_COLOR = (0, 255, 0)
BOX_THICKNESS = 2
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
FONT_THICKNESS = 1

# Inference settings
WARMUP_DURATION = 5.0
INFERENCES_PER_SECOND = 3
INFERENCE_INTERVAL = 1.0 / INFERENCES_PER_SECOND

# Shared state between threads
latest_frame = None
latest_frame_lock = threading.Lock()
inference_results = []  # list of (frame_copy, predictions) tuples
results_lock = threading.Lock()
inference_done = threading.Event()
start_inference = threading.Event()


def draw_predictions(frame, predictions):
    """Draw bounding boxes and labels on a frame."""
    for pred in predictions:
        x1 = int(pred['x'] - pred['width'] / 2)
        y1 = int(pred['y'] - pred['height'] / 2)
        x2 = int(pred['x'] + pred['width'] / 2)
        y2 = int(pred['y'] + pred['height'] / 2)
        label = f"{pred['class']} {pred['confidence']:.0%}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, BOX_THICKNESS)

        (text_w, text_h), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
        cv2.rectangle(frame, (x1, y1 - text_h - baseline - 4), (x1 + text_w, y1), TEXT_BG_COLOR, -1)
        cv2.putText(frame, label, (x1, y1 - baseline - 2), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)


def inference_worker():
    """Background thread: waits for signal, runs 3 inferences over ~1s, stores each result."""
    start_inference.wait()

    for i in range(INFERENCES_PER_SECOND):
        t0 = time.monotonic()

        # Grab the latest frame
        with latest_frame_lock:
            frame = latest_frame

        if frame is None:
            time.sleep(INFERENCE_INTERVAL)
            continue

        frame_copy = frame.copy()

        # Run inference against local server
        try:
            print(f"Running inference {i + 1}/{INFERENCES_PER_SECOND}...")
            preds = run_inference(frame)
        except Exception as e:
            print(f"Inference error: {e}")
            preds = []

        with results_lock:
            inference_results.append((frame_copy, preds))

        # Sleep for the remainder of the interval
        elapsed = time.monotonic() - t0
        sleep_time = INFERENCE_INTERVAL - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("Inference complete.")
    inference_done.set()


# Start the inference thread (it will wait for the signal)
inference_thread = threading.Thread(target=inference_worker, daemon=True)
inference_thread.start()

print("Camera feed started. Warming up for 2 seconds...")

# --- Phase 1: 2-second warm-up with live view, no predictions ---
warmup_start = time.monotonic()
quit_early = False

while time.monotonic() - warmup_start < WARMUP_DURATION:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from camera.")
        quit_early = True
        break

    with latest_frame_lock:
        latest_frame = frame.copy()

    cv2.imshow("ArduCam - Live", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        quit_early = True
        break

if not quit_early:
    # --- Phase 2: 1-second inference with live view ---
    print("Running inference for ~1 second...")
    start_inference.set()

    while not inference_done.is_set():
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame from camera.")
            break

        with latest_frame_lock:
            latest_frame = frame.copy()

        cv2.imshow("ArduCam - Live", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit_early = True
            break

# Done capturing
cap.release()
inference_thread.join(timeout=5)
cv2.destroyAllWindows()

# --- Phase 3: Average predictions and output final result ---
if not quit_early:
    with results_lock:
        results = list(inference_results)

    if not results:
        print("No predictions were captured.")
    else:
        # Bounding boxes
        print(f"Saving and displaying {len(results)} prediction(s)...")

        for i, (frame_copy, preds) in enumerate(results):
            draw_predictions(frame_copy, preds)
            det_count = len(preds)
            title = f"Prediction {i + 1}: {det_count} detection(s)"
            cv2.putText(frame_copy, title, (10, 25), FONT, 0.7, (0, 0, 255), 2)

            filename = f"prediction_{i + 1}.png"
            cv2.imwrite(filename, frame_copy)
            print(f"  Saved {filename} — {det_count} detection(s)")

            cv2.imshow(f"Prediction {i + 1}/{len(results)}", frame_copy)

        # Collect all predictions across the 3 inferences and group by class
        class_confidences = {}
        for _, preds in results:
            for pred in preds:
                cls = pred['class']
                conf = pred['confidence']
                class_confidences.setdefault(cls, []).append(conf)

        if not class_confidences:
            print("No detections across all inferences.")
            final_class = "none"
            final_confidence = 0.0
        else:
            # Average confidence per class, pick the class with highest average
            class_avg = {}
            for cls, confs in class_confidences.items():
                class_avg[cls] = sum(confs) / len(confs)

            final_class = max(class_avg, key=class_avg.get)
            final_confidence = class_avg[final_class]

            print(f"Averaged predictions across {len(results)} inferences:")
            for cls, avg in sorted(class_avg.items(), key=lambda x: -x[1]):
                count = len(class_confidences[cls])
                print(f"  {cls}: avg confidence {avg:.2%} ({count} detection(s))")

        print(f"\nFinal prediction: {final_class} ({final_confidence:.2%})")

        # Write result to text file
        output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "prediction_result.txt")
        with open(output_path, "w") as f:
            f.write(f"{final_class},{final_confidence:.4f}\n")
        print(f"Result saved to {output_path}")

        # Display the last captured frame with the final prediction overlay
        final_frame = results[-1][0].copy()
        label = f"Final: {final_class} {final_confidence:.0%}"
        cv2.putText(final_frame, label, (10, 450), FONT, 0.7, (0, 0, 255), 2)
        draw_predictions(final_frame, results[-1][1])
        cv2.imwrite("prediction_final.png", final_frame)
        cv2.imshow("Final Prediction", final_frame)

        print("Press 'q' to quit.")
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()
print("Done.")

# FIXME: automatically end script or allow user to make more predictions before manual keypress to terminate
