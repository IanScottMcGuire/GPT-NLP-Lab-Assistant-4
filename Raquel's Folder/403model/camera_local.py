import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import base64
import cv2
import numpy as np
import requests
import threading

# --- Local inference server configuration ---
INFERENCE_SERVER = "http://localhost:9001"
MODEL_ID = "components-model-v2tgx/5"
CONFIDENCE = 0.40
IOU_THRESHOLD = 0.30

INFERENCE_URL = f"{INFERENCE_SERVER}/infer/object_detection"

# Drawing settings
BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)
TEXT_BG_COLOR = (0, 255, 0)
BOX_THICKNESS = 2
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
FONT_THICKNESS = 1

# Shared state between threads
latest_frame = None
latest_frame_lock = threading.Lock()
latest_preds = []
preds_lock = threading.Lock()
inference_running = threading.Event()


def letterbox(frame, size=640):
    """Pad frame to a square with black bars, preserving aspect ratio.
    Returns (padded_frame, scale, pad_left, pad_top).
    """
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(frame, (new_w, new_h))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    pad_top = (size - new_h) // 2
    pad_left = (size - new_w) // 2
    canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
    return canvas, scale, pad_left, pad_top


def unletterbox_preds(preds, scale, pad_left, pad_top):
    """Convert prediction coordinates from letterboxed space back to original frame space."""
    adjusted = []
    for p in preds:
        adjusted.append({
            **p,
            'x': (p['x'] - pad_left) / scale,
            'y': (p['y'] - pad_top) / scale,
            'width': p['width'] / scale,
            'height': p['height'] / scale,
        })
    return adjusted


def run_inference(frame):
    """POST a BGR frame to the local Roboflow inference server and return predictions list."""
    square, scale, pad_left, pad_top = letterbox(frame)
    _, buf = cv2.imencode(".jpg", cv2.cvtColor(square, cv2.COLOR_BGR2RGB))
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
        headers={"Expect": ""},
        timeout=10,
    )
    resp.raise_for_status()
    preds = resp.json().get("predictions", [])
    return unletterbox_preds(preds, scale, pad_left, pad_top)


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
    """Background thread: continuously runs inference on the latest frame."""
    saved_debug_frame = False
    while True:
        with latest_frame_lock:
            frame = latest_frame

        if frame is None:
            continue

        if not saved_debug_frame:
            cv2.imwrite("debug_sent_frame.jpg", frame)
            print("Saved debug_sent_frame.jpg — check this is what you expect the model to see.")
            saved_debug_frame = True

        try:
            preds = run_inference(frame)
        except Exception as e:
            print(f"Inference error: {e}")
            preds = []

        with preds_lock:
            latest_preds.clear()
            latest_preds.extend(preds)

        if preds:
            scores = ", ".join(f"{p['class']} {p['confidence']:.0%}" for p in preds)
            print(f"Detections: {scores}")
        else:
            print("No detections.")

        inference_running.clear()
        inference_running.wait()  # pause until main thread signals the next frame is ready


# Open the camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera. Try a different index (1, 2, etc.).")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

infer_thread = threading.Thread(target=inference_worker, daemon=True)
infer_thread.start()

print("Live inference running. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from camera.")
        break

    with latest_frame_lock:
        latest_frame = frame.copy()

    # Signal inference thread to pick up the new frame
    inference_running.set()

    # Draw the most recent predictions on the live frame
    with preds_lock:
        preds_snapshot = list(latest_preds)

    display = frame.copy()
    draw_predictions(display, preds_snapshot)
    cv2.imshow("Live Inference", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Done.")
