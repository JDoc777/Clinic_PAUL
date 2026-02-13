from picamera2 import Picamera2
from ultralytics import YOLO
import threading
import torch
import time

# -------- SETTINGS --------
CAMERA_SIZE = (640, 480)
YOLO_SIZE = 256        # smaller = less heat
CONF_THRES = 0.35
MODEL_PATH = "yolov8n.pt"
# --------------------------

# Limit CPU usage (VERY IMPORTANT)
torch.set_num_threads(1)

latest_frame = None
running = True
lock = threading.Lock()

print("Loading YOLO...")
model = YOLO(MODEL_PATH)
print("Model loaded.")

# ----------------------------
# YOLO Worker Thread
# ----------------------------
def yolo_worker():
    global latest_frame, running

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with lock:
            frame = latest_frame.copy()

        results = model(frame, imgsz=256, verbose=False)
        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0:
            class_counts = {}

            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                # Count per class
                if label not in class_counts:
                    class_counts[label] = 0

                class_counts[label] += 1
                index = class_counts[label]

                print(f"{label.capitalize()} {index} | Confidence: {conf:.2f}")

        time.sleep(0.02)



# ----------------------------
# Main
# ----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
picam2.configure(config)
picam2.start()

thread = threading.Thread(target=yolo_worker)
thread.start()

try:
    while True:
        frame = picam2.capture_array()

        # Fix channel layout (working version)
        frame = frame[:, :, :3]
        frame = frame[:, :, ::-1]  # BGR → RGB

        with lock:
            latest_frame = frame

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    thread.join()
    picam2.stop()
