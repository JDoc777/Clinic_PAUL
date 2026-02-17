from picamera2 import Picamera2
from ultralytics import YOLO
import threading
import torch
import time
from collections import deque
import matplotlib.pyplot as plt

# -------- SETTINGS --------
CAMERA_SIZE = (640, 480)
YOLO_SIZE = 256
CONF_THRES = 0.35
MODEL_PATH = "yolov8n.pt"

HFOV_DEG = 102.0          # camera horizontal field of view
CAMERA_YAW_OFFSET_DEG = 0 # assume camera points forward
PLOT_WINDOW_SEC = 10.0    # seconds of history in plot
# --------------------------

torch.set_num_threads(1)

latest_frame = None
running = True
lock = threading.Lock()

# shared plot data
angle_lock = threading.Lock()
t_hist = deque()
a_hist = deque()

def pixel_center_to_angle_deg(cx, img_w, hfov_deg):
    """
    Map pixel center cx to yaw angle (deg) relative to camera forward axis.
    Left side negative, right side positive.
    """
    x_norm = (cx - (img_w / 2.0)) / (img_w / 2.0)  # [-1, +1]
    return x_norm * (hfov_deg / 2.0)

print("Loading YOLO...")
model = YOLO(MODEL_PATH)
print("Model loaded.")

# ----------------------------
# YOLO Worker Thread
# ----------------------------
def yolo_worker():
    global latest_frame, running

    img_w = CAMERA_SIZE[0]

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with lock:
            frame = latest_frame.copy()

        results = model(frame, imgsz=YOLO_SIZE, verbose=False)
        boxes = results[0].boxes

        best = None  # (conf, label, angle_deg)
        class_counts = {}

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                # bbox xyxy in pixels
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = 0.5 * (x1 + x2)

                angle_deg = pixel_center_to_angle_deg(cx, img_w, HFOV_DEG) + CAMERA_YAW_OFFSET_DEG

                # Count per class (so you can label "Person 1", "Person 2", etc.)
                class_counts[label] = class_counts.get(label, 0) + 1
                index = class_counts[label]

                print(f"{label.capitalize()} {index} | conf={conf:.2f} | cx={cx:.1f}px | angle={angle_deg:.1f}°")

                if best is None or conf > best[0]:
                    best = (conf, label, angle_deg)

        # push best angle into plot history
        if best is not None:
            now = time.time()
            with angle_lock:
                t_hist.append(now)
                a_hist.append(best[2])

                # trim history
                while t_hist and (now - t_hist[0]) > PLOT_WINDOW_SEC:
                    t_hist.popleft()
                    a_hist.popleft()

        time.sleep(0.02)

# ----------------------------
# Plot Thread (Live)
# ----------------------------
def plot_worker():
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], marker='o', linestyle='-')

    ax.set_title("Object Angle vs Time (best detection)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_ylim(-HFOV_DEG/2.0 - 5, HFOV_DEG/2.0 + 5)
    ax.grid(True)

    t0 = time.time()

    while running:
        with angle_lock:
            if len(t_hist) >= 2:
                xs = [t - t0 for t in t_hist]
                ys = list(a_hist)
            else:
                xs, ys = [], []

        line.set_data(xs, ys)

        if xs:
            ax.set_xlim(max(0, xs[-1] - PLOT_WINDOW_SEC), xs[-1] + 0.5)
        else:
            ax.set_xlim(0, PLOT_WINDOW_SEC)

        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

# ----------------------------
# Main
# ----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
picam2.configure(config)
picam2.start()

yolo_thread = threading.Thread(target=yolo_worker, daemon=True)
yolo_thread.start()

plot_thread = threading.Thread(target=plot_worker, daemon=True)
plot_thread.start()

try:
    while True:
        frame = picam2.capture_array()

        # keep RGB channels only
        frame = frame[:, :, :3]

        # If your detections look off, try commenting out the next line:
        frame = frame[:, :, ::-1]  # BGR → RGB (depends on your camera output)

        with lock:
            latest_frame = frame

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    yolo_thread.join(timeout=1.0)
    picam2.stop()
