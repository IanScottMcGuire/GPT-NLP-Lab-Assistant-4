import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import base64
import cv2
import numpy as np
import requests
import threading
import time

# ── Headless mode: set when called from the UI via subprocess ──
# In headless mode: no cv2.imshow, no cv2.waitKey, exit after one inference.
# Run directly from terminal for live display mode.
HEADLESS = os.environ.get("HEADLESS") == "1"

# ── Inference server (Roboflow Docker container on Jetson) ─────
INFERENCE_SERVER = "http://localhost:9001"
MODEL_ID         = "components-model-v2tgx/5"
CONFIDENCE       = 0.40
IOU_THRESHOLD    = 0.30
INFERENCE_URL    = f"{INFERENCE_SERVER}/infer/object_detection"

# ── Result output ──────────────────────────────────────────────
RESULT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "prediction_result.txt")

# ── Drawing settings (live mode only) ─────────────────────────
BOX_COLOR    = (0, 255, 0)
TEXT_COLOR   = (255, 255, 255)
TEXT_BG_COLOR = (0, 255, 0)
BOX_THICKNESS = 2
FONT          = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE    = 0.6
FONT_THICKNESS = 1

# ── Shared state ───────────────────────────────────────────────
latest_frame      = None
latest_frame_lock = threading.Lock()
latest_preds      = []
preds_lock        = threading.Lock()
inference_running = threading.Event()


def letterbox(frame, size=640):
    h, w = frame.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(frame, (new_w, new_h))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    pad_top  = (size - new_h) // 2
    pad_left = (size - new_w) // 2
    canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
    return canvas, scale, pad_left, pad_top


def unletterbox_preds(preds, scale, pad_left, pad_top):
    adjusted = []
    for p in preds:
        adjusted.append({
            **p,
            'x':      (p['x'] - pad_left) / scale,
            'y':      (p['y'] - pad_top)  / scale,
            'width':   p['width']  / scale,
            'height':  p['height'] / scale,
        })
    return adjusted


def run_inference(frame):
    square, scale, pad_left, pad_top = letterbox(frame)
    _, buf = cv2.imencode(".jpg", cv2.cvtColor(square, cv2.COLOR_BGR2RGB))
    b64 = base64.b64encode(buf).decode("utf-8")
    resp = requests.post(
        INFERENCE_URL,
        json={
            "id":            "camera_local",
            "api_key":       os.getenv("RoboFlow_KEY"),
            "model_id":      MODEL_ID,
            "image":         {"type": "base64", "value": b64},
            "confidence":    CONFIDENCE,
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
        x1 = int(pred['x'] - pred['width']  / 2)
        y1 = int(pred['y'] - pred['height'] / 2)
        x2 = int(pred['x'] + pred['width']  / 2)
        y2 = int(pred['y'] + pred['height'] / 2)
        label = f"{pred['class']} {pred['confidence']:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, BOX_THICKNESS)
        (text_w, text_h), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
        cv2.rectangle(frame, (x1, y1 - text_h - baseline - 4), (x1 + text_w, y1), TEXT_BG_COLOR, -1)
        cv2.putText(frame, label, (x1, y1 - baseline - 2), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)


def write_result(final_class, final_confidence):
    with open(RESULT_FILE, "w") as f:
        f.write(f"{final_class},{final_confidence:.4f}\n")
    print(f"Result saved to {RESULT_FILE}")


# ══════════════════════════════════════════════════════════════
#   HEADLESS MODE — single shot, no display, exits when done
# ══════════════════════════════════════════════════════════════

def run_headless():
    WARMUP_SECONDS = 5.0
    NUM_INFERENCES = 3

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        os._exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print(f"Warming up camera for {WARMUP_SECONDS}s...")
    warmup_start = time.time()
    last_frame = None
    while time.time() - warmup_start < WARMUP_SECONDS:
        ret, frame = cap.read()
        if ret:
            last_frame = frame.copy()
        time.sleep(0.03)

    if last_frame is None:
        print("Error: No frames captured during warm-up.")
        cap.release()
        os._exit(1)

    print(f"Running {NUM_INFERENCES} inferences...")
    all_preds = []
    interval = 1.0 / NUM_INFERENCES
    for i in range(NUM_INFERENCES):
        t0 = time.time()
        ret, frame = cap.read()
        if ret:
            last_frame = frame.copy()
        try:
            preds = run_inference(last_frame)
            all_preds.extend(preds)
            print(f"  Inference {i+1}/{NUM_INFERENCES}: {len(preds)} detection(s)")
        except Exception as e:
            print(f"  Inference {i+1} error: {e}")
        elapsed = time.time() - t0
        if interval - elapsed > 0:
            time.sleep(interval - elapsed)

    cap.release()

    if not all_preds:
        print("No components detected.")
        write_result("none", 0.0)
        os._exit(0)

    # Average confidence per class, pick highest
    class_confs = {}
    for pred in all_preds:
        class_confs.setdefault(pred["class"], []).append(pred["confidence"])
    class_avg     = {cls: sum(v) / len(v) for cls, v in class_confs.items()}
    final_class   = max(class_avg, key=class_avg.get)
    final_conf    = class_avg[final_class]

    print(f"Final prediction: {final_class} ({final_conf:.0%})")
    write_result(final_class, final_conf)
    os._exit(0)


# ══════════════════════════════════════════════════════════════
#   LIVE MODE — continuous display, press q to quit
# ══════════════════════════════════════════════════════════════

def inference_worker():
    saved_debug_frame = False
    while True:
        with latest_frame_lock:
            frame = latest_frame
        if frame is None:
            continue
        if not saved_debug_frame:
            cv2.imwrite("debug_sent_frame.jpg", frame)
            print("Saved debug_sent_frame.jpg")
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
        inference_running.wait()


def run_live():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    infer_thread = threading.Thread(target=inference_worker, daemon=True)
    infer_thread.start()

    print("Live inference running. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break
        with latest_frame_lock:
            latest_frame_ref = frame.copy()
        # update shared latest_frame
        globals()['latest_frame'] = latest_frame_ref
        inference_running.set()
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


# ══════════════════════════════════════════════════════════════
#   ENTRY POINT
# ══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if HEADLESS:
        run_headless()
    else:
        run_live()
