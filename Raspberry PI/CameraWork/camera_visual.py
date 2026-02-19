from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
import threading
import time
import torch
torch.set_num_threads(1)

# -------- SETTINGS --------
CAMERA_SIZE = (640, 480)
YOLO_SIZE = 192
CONF_THRES = 0.35
MODEL_PATH = "yolov8n.pt"
# --------------------------

# Shared variables between threads
latest_frame = None
latest_result = None
running = True
lock = threading.Lock()

print("Loading YOLO...")
model = YOLO(MODEL_PATH)
print("Model loaded.")


# ----------------------------
# YOLO Thread
# ----------------------------
def yolo_worker():
    global latest_frame, latest_result, running

    last_processed_id = 0

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with lock:
            frame = latest_frame.copy()

        small = cv2.resize(frame, (YOLO_SIZE, YOLO_SIZE))

        results = model(small, verbose=False)

        annotated_small = results[0].plot()
        annotated = cv2.resize(annotated_small, CAMERA_SIZE)

        with lock:
            latest_result = annotated

        # Give CPU breathing room
        time.sleep(0.01)



# ----------------------------
# Main
# ----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
picam2.configure(config)
picam2.start()

# Start YOLO thread
thread = threading.Thread(target=yolo_worker)
thread.start()

fps_t0 = time.time()
fps_frames = 0
fps = 0

try:
    while True:
        frame = picam2.capture_array()

        # Fix channel layout
        frame = frame[:, :, :3]
        frame = frame[:, :, ::-1]  # BGR → RGB

        with lock:
            latest_frame = frame

        display = frame.copy()

        with lock:
            if latest_result is not None:
                display = latest_result.copy()

        # FPS counter (camera FPS)
        fps_frames += 1
        now = time.time()
        if now - fps_t0 >= 1.0:
            fps = fps_frames / (now - fps_t0)
            fps_frames = 0
            fps_t0 = now

        cv2.putText(
            display,
            f"Camera FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow("Threaded YOLO", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    running = False
    thread.join()
    picam2.stop()
    cv2.destroyAllWindows()
