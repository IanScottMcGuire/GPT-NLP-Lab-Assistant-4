import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import csv
import sys
import time
import threading
from roboflow import Roboflow

# --- Roboflow setup ---
rf = Roboflow(api_key="hFyT16eNtzwRsmbWtu97")
project = rf.workspace("gptnlplabassistant4").project("components-model-v2tgx")
model = project.version("5").model

# --- CSV settings ---
CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "video_inference.csv")
CLASSES = [
    "Capacitor", "Ceramic Capacitor", "Diode", "IC", "LED",
    "Photodiode", "Potentiometer", "Resistor", "Transistor"
]
ROWS_PER_CLASS = 20

# --- Inference settings ---
NUM_CAPTURES = 4
CAPTURE_INTERVAL = 0.5  # seconds between each capture (4 × 0.5s = 2s total)

# --- Camera settings ---
CAMERA_INDEX = 2

# --- Shared frame state (written by main loop, read by inference thread) ---
latest_frame = None
latest_frame_lock = threading.Lock()

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def generate_csv(path):
    """Create video_inference.csv with actual column pre-filled, others blank."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['actual', 'predicted', 'confidence'])
        for cls in CLASSES:
            for _ in range(ROWS_PER_CLASS):
                writer.writerow([cls, '', ''])
    print(f"Generated {path}")


def read_csv(path):
    with open(path, 'r', newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['actual', 'predicted', 'confidence'])
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# Inference
# ---------------------------------------------------------------------------

def run_inference_sequence():
    """Capture 4 frames at 0.5 s intervals, run Roboflow on each, return averaged result.

    Returns:
        (predicted_class, avg_confidence) or (None, 0.0) if no detections.
    """
    all_preds = []

    for i in range(NUM_CAPTURES):
        t0 = time.monotonic()

        with latest_frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is not None:
            try:
                print(f"  Capture {i + 1}/{NUM_CAPTURES}...")
                result = model.predict(frame, confidence=40, overlap=30).json()
                preds = result.get('predictions', [])
                all_preds.extend(preds)
                print(f"    {len(preds)} detection(s)")
            except Exception as e:
                print(f"  Inference error on capture {i + 1}: {e}")
        else:
            print(f"  Capture {i + 1}: no frame available, skipping.")

        elapsed = time.monotonic() - t0
        remaining = CAPTURE_INTERVAL - elapsed
        if remaining > 0 and i < NUM_CAPTURES - 1:
            time.sleep(remaining)

    if not all_preds:
        return None, 0.0

    # Average confidence per class across all captures; pick highest
    class_confidences = {}
    for pred in all_preds:
        cls = pred['class']
        conf = pred['confidence']
        class_confidences.setdefault(cls, []).append(conf)

    class_avg = {cls: sum(confs) / len(confs) for cls, confs in class_confidences.items()}

    print("  Averaged confidences:")
    for cls, avg in sorted(class_avg.items(), key=lambda x: -x[1]):
        print(f"    {cls}: {avg:.2%} ({len(class_confidences[cls])} detection(s))")

    final_class = max(class_avg, key=class_avg.get)
    final_confidence = class_avg[final_class]
    return final_class, final_confidence


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global latest_frame

    # Step 1: Generate CSV
    generate_csv(CSV_PATH)
    rows = read_csv(CSV_PATH)
    total_rows = len(rows)

    # Step 2: Open camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Error: Could not open camera. Try a different index.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Warm up camera (discard first few frames)
    print("Warming up camera...")
    for _ in range(15):
        cap.read()
    print("Camera ready.\n")

    row_idx = 0
    quit_flag = False

    while row_idx < total_rows and not quit_flag:
        current_class = rows[row_idx]['actual']
        print(f"[{row_idx + 1}/{total_rows}] Current class: {current_class}")
        print("  Press 'f' in camera window to start inference, 'q' to quit.")

        # --- Wait for 'f' or 'q' while showing live view ---
        waiting = True
        while waiting and not quit_flag:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame.")
                continue

            with latest_frame_lock:
                latest_frame = frame.copy()

            display = frame.copy()
            cv2.putText(display, f"Testing: {current_class}", (10, 30),
                        FONT, 0.7, (0, 255, 0), 2)
            cv2.putText(display, "f = infer  |  q = quit", (10, 60),
                        FONT, 0.55, (200, 200, 200), 1)
            cv2.imshow("ArduCam - Live", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('f'):
                waiting = False
            elif key == ord('q'):
                quit_flag = True

        if quit_flag:
            break

        # --- Run inference sequence in background while live view continues ---
        print(f"  Starting 2 s inference sequence...")
        inference_output = [None, 0.0]
        inference_done = threading.Event()

        def do_inference():
            cls, conf = run_inference_sequence()
            inference_output[0] = cls
            inference_output[1] = conf
            inference_done.set()

        infer_thread = threading.Thread(target=do_inference, daemon=True)
        infer_thread.start()

        while not inference_done.is_set():
            ret, frame = cap.read()
            if ret:
                with latest_frame_lock:
                    latest_frame = frame.copy()
                display = frame.copy()
                cv2.putText(display, f"Inferring: {current_class}...", (10, 30),
                            FONT, 0.7, (0, 165, 255), 2)
                cv2.imshow("ArduCam - Live", display)
            cv2.waitKey(1)

        infer_thread.join()

        final_class, final_confidence = inference_output[0], inference_output[1]

        if final_class is None:
            print(f"  No detections — row left blank.")
            rows[row_idx]['predicted'] = ''
            rows[row_idx]['confidence'] = ''
        else:
            print(f"  Final prediction: {final_class} ({final_confidence:.2%})")
            rows[row_idx]['predicted'] = final_class
            rows[row_idx]['confidence'] = f"{final_confidence:.4f}"

        write_csv(CSV_PATH, rows)
        row_idx += 1

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

    if quit_flag:
        print("\nExited by user.")
    else:
        print(f"\nAll {total_rows} rows exhausted. Done.")


if __name__ == "__main__":
    main()
