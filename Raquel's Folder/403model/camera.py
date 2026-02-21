import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import threading
import time
from roboflow import Roboflow

# Initialize Roboflow model
rf = Roboflow(api_key="hFyT16eNtzwRsmbWtu97")
project = rf.workspace("gptnlplabassistant4").project("components-model-v2tgx")
model = project.version("4").model

# Open the ArduCam (USB camera)
cap = cv2.VideoCapture(2)
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
    """Draw raw Roboflow predictions on a frame."""
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

        # Run inference
        try:
            print(f"Running inference {i + 1}/{INFERENCES_PER_SECOND}...")
            result = model.predict(frame, confidence=40, overlap=30).json()
            preds = result.get('predictions', [])
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

# --- Phase 3: Display each prediction as a separate image ---
if not quit_early:
    with results_lock:
        results = list(inference_results)

    if not results:
        print("No predictions were captured.")
    else:
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

        print("Press 'q' to quit.")
        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()
print("Done.")
