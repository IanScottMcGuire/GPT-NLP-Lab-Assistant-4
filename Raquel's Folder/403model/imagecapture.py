import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time

# --- Settings ---
CAMERA_INDEX = 2
BRIGHTNESS = -40  # Adjust this value to control exposure/brightness (0-100 typical range)
RESOLUTION = (640, 640)

# Output directory
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "image_captures")
os.makedirs(SAVE_DIR, exist_ok=True)

# Open ArduCam
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print("Error: Could not open camera. Try a different index (1, 2, etc.).")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
cap.set(cv2.CAP_PROP_BRIGHTNESS, BRIGHTNESS)

print(f"Camera opened (index {CAMERA_INDEX}), brightness set to {BRIGHTNESS}.")
print("Press SPACEBAR to capture an image. Press 'q' to quit.")

capture_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame from camera.")
        break

    cv2.imshow("ArduCam - Live (Space=Capture, Q=Quit)", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        capture_count += 1
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"capture_{timestamp}_{capture_count}.png"
        filepath = os.path.join(SAVE_DIR, filename)
        cv2.imwrite(filepath, frame)
        print(f"Saved: {filepath}")

cap.release()
cv2.destroyAllWindows()
print(f"Done. {capture_count} image(s) captured.")
