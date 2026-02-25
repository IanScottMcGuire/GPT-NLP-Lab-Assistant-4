import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import base64
import time
import threading
from collections import defaultdict

import cv2
import numpy as np
import requests

# --- Local inference server configuration ---
INFERENCE_SERVER = "http://localhost:9001"
MODEL_ID = "components-model-v2tgx/5"
CONFIDENCE = 0.50
IOU_THRESHOLD = 0.50
INFERENCE_URL = f"{INFERENCE_SERVER}/infer/object_detection"

WARMUP_DURATION = 3.0            # seconds, camera on but no inference
INFERENCE_DURATION = 3.0         # seconds, collect predictions
AVERAGING_DISPLAY_DURATION = 1.5 # seconds to show overlay before closing

OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "prediction_result.txt")

# Drawing settings
BOX_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)
TEXT_BG_COLOR = (0, 255, 0)
BOX_THICKNESS = 2
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
FONT_THICKNESS = 1

# --- Shared state ---
latest_frame = None
latest_frame_lock = threading.Lock()
latest_preds = []          # most recent predictions for live display
preds_lock = threading.Lock()
collected_preds = []       # one list of predictions per completed inference call
collected_lock = threading.Lock()
stop_inference = threading.Event()


# --- Helper functions ---

def letterbox(frame, size=640):
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
    return [
        {
            **p,
            'x': (p['x'] - pad_left) / scale,
            'y': (p['y'] - pad_top) / scale,
            'width': p['width'] / scale,
            'height': p['height'] / scale,
        }
        for p in preds
    ]


def run_inference(frame):
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


def put_overlay(frame, text):
    """Draw a semi-transparent black banner with yellow text at the top of frame."""
    h, w = frame.shape[:2]
    banner = frame.copy()
    cv2.rectangle(banner, (0, 0), (w, 52), (0, 0, 0), -1)
    cv2.addWeighted(banner, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, text, (10, 36), FONT, 0.85, (0, 255, 255), 2)


def average_predictions(all_preds, total_frames):
    """
    For each class, sum confidence across all inference frames, then divide by
    total_frames (not just frames with that class) so classes that only appeared
    occasionally are penalised.  Returns (best_class, avg_confidence).
    """
    class_conf_sum = defaultdict(float)
    for frame_preds in all_preds:
        for p in frame_preds:
            class_conf_sum[p['class']] += p['confidence']

    if not class_conf_sum or total_frames == 0:
        return None, 0.0

    class_avg = {cls: total / total_frames for cls, total in class_conf_sum.items()}
    best_class = max(class_avg, key=class_avg.get)
    return best_class, class_avg[best_class]


# --- Inference background thread ---

def inference_worker():
    """Continuously grabs the latest frame, runs inference, and appends results."""
    while not stop_inference.is_set():
        with latest_frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            time.sleep(0.01)
            continue

        try:
            preds = run_inference(frame)
        except Exception as e:
            print(f"Inference error: {e}")
            preds = []

        with collected_lock:
            collected_preds.append(preds)

        with preds_lock:
            latest_preds.clear()
            latest_preds.extend(preds)

        if preds:
            scores = ", ".join(f"{p['class']} {p['confidence']:.0%}" for p in preds)
            print(f"  Detections: {scores}")
        else:
            print("  No detections this frame.")


# --- Open camera in a background thread so the window appears immediately ---
cap = None
cam_ready = threading.Event()

def open_camera():
    global cap
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cam_ready.set()

cam_thread = threading.Thread(target=open_camera, daemon=True)
cam_thread.start()

# Show a placeholder window immediately while the camera initialises
placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
print("Opening camera...")
while not cam_ready.is_set():
    display = placeholder.copy()
    put_overlay(display, "Camera initializing...")
    cv2.imshow("Live Inference", display)
    if cv2.waitKey(100) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        exit(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    cv2.destroyAllWindows()
    exit(1)

# ── Phase 1: Warmup (no inference) ──────────────────────────────────────────
print(f"[Phase 1] Warming up camera for {WARMUP_DURATION:.0f}s...")
warmup_end = time.time() + WARMUP_DURATION

while time.time() < warmup_end:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    with latest_frame_lock:
        latest_frame = frame.copy()

    display = frame.copy()
    remaining = max(0.0, warmup_end - time.time())
    put_overlay(display, f"Warming up... {remaining:.1f}s")
    cv2.imshow("Live Inference", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

# ── Phase 2: Inference collection (3s) ──────────────────────────────────────
print(f"[Phase 2] Collecting inference for {INFERENCE_DURATION:.0f}s...")
infer_thread = threading.Thread(target=inference_worker, daemon=True)
infer_thread.start()

inference_end = time.time() + INFERENCE_DURATION

while time.time() < inference_end:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    with latest_frame_lock:
        latest_frame = frame.copy()

    with preds_lock:
        preds_snapshot = list(latest_preds)

    display = frame.copy()
    draw_predictions(display, preds_snapshot)
    remaining = max(0.0, inference_end - time.time())
    put_overlay(display, f"Inferring... {remaining:.1f}s remaining")
    cv2.imshow("Live Inference", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop_inference.set()
        infer_thread.join(timeout=3)
        cap.release()
        cv2.destroyAllWindows()
        exit(0)

# Stop inference thread and collect results
stop_inference.set()
infer_thread.join(timeout=5)

with collected_lock:
    snap = list(collected_preds)
total_frames = len(snap)
print(f"[Phase 2] Collected {total_frames} inference frame(s).")

# ── Phase 3: Averaging overlay (camera still open) ──────────────────────────
print("[Phase 3] Averaging predictions...")
best_class, avg_conf = average_predictions(snap, total_frames)

averaging_end = time.time() + AVERAGING_DISPLAY_DURATION
while time.time() < averaging_end:
    ret, frame = cap.read()
    if not ret:
        break
    display = frame.copy()
    put_overlay(display, f"Averaging {total_frames} prediction frame(s)...")
    cv2.imshow("Live Inference", display)
    cv2.waitKey(1)

# ── Write result ─────────────────────────────────────────────────────────────
if best_class:
    result_line = f"{best_class} {avg_conf:.2%}"
else:
    result_line = "No detections"

with open(OUTPUT_FILE, "w") as f:
    f.write(result_line + "\n")

print(f"[Result] {result_line}")
print(f"[Output] Written to {OUTPUT_FILE}")

# ── Cleanup ───────────────────────────────────────────────────────────────────
cap.release()
cv2.destroyAllWindows()
print("Done.")
