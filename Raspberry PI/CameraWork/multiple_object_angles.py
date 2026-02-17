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

HFOV_DEG = 102.0
CAMERA_YAW_OFFSET_DEG = 0.0
PLOT_WINDOW_SEC = 10.0

PRINT_DELTA_DEG = 0.5   # only print if range changes by >= this amount
PRINT_HZ = 10           # max print updates per second (prevents spam)
# --------------------------

torch.set_num_threads(1)

latest_frame = None
running = True
frame_lock = threading.Lock()

# Each entry: (timestamp, [ {key, label, conf, a_min, a_max, a_w}, ... ])
snap_lock = threading.Lock()
snapshots = deque()

def pixel_x_to_angle_deg(x, img_w, hfov_deg):
    x_norm = (x - (img_w / 2.0)) / (img_w / 2.0)  # [-1, +1]
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

    last_printed = {}   # key -> (a_min, a_max)
    last_print_time = 0.0

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with frame_lock:
            frame = latest_frame.copy()

        results = model(frame, imgsz=YOLO_SIZE, verbose=False)
        boxes = results[0].boxes

        detections = []
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

                angle_left  = pixel_x_to_angle_deg(x1, img_w, HFOV_DEG) + CAMERA_YAW_OFFSET_DEG
                angle_right = pixel_x_to_angle_deg(x2, img_w, HFOV_DEG) + CAMERA_YAW_OFFSET_DEG

                a_min = min(angle_left, angle_right)
                a_max = max(angle_left, angle_right)
                a_w = a_max - a_min

                class_counts[label] = class_counts.get(label, 0) + 1
                idx = class_counts[label]
                key = f"{label}_{idx}"  # temporary key per-frame (see note below)

                detections.append({
                    "key": key,
                    "label": label,
                    "conf": conf,
                    "a_min": a_min,
                    "a_max": a_max,
                    "a_w": a_w
                })

        # print updates (rate limited + only when changed enough)
        now = time.time()
        if now - last_print_time >= (1.0 / PRINT_HZ):
            last_print_time = now

            # show a compact "frame update" header if anything is detected
            if len(detections) > 0:
                # We'll print each detection only if its span changed enough vs last time we printed it
                for d in detections:
                    key = d["key"]
                    a_min = d["a_min"]
                    a_max = d["a_max"]

                    prev = last_printed.get(key)
                    if prev is None:
                        # first time seeing this key
                        print(f"{d['label'].capitalize()} [{key}] span {a_min:.1f}° .. {a_max:.1f}°  (w={d['a_w']:.1f}°, conf={d['conf']:.2f})")
                        last_printed[key] = (a_min, a_max)
                    else:
                        prev_min, prev_max = prev
                        if abs(a_min - prev_min) >= PRINT_DELTA_DEG or abs(a_max - prev_max) >= PRINT_DELTA_DEG:
                            print(f"{d['label'].capitalize()} [{key}] span {a_min:.1f}° .. {a_max:.1f}°  (w={d['a_w']:.1f}°, conf={d['conf']:.2f})")
                            last_printed[key] = (a_min, a_max)

        # store snapshot for plotting
        with snap_lock:
            snapshots.append((now, detections))
            while snapshots and (now - snapshots[0][0]) > PLOT_WINDOW_SEC:
                snapshots.popleft()

        time.sleep(0.02)

# ----------------------------
# Plot Thread (Live)
# ----------------------------
def plot_worker():
    plt.ion()
    fig, ax = plt.subplots()
    t0 = time.time()

    while running:
        with snap_lock:
            snap_copy = list(snapshots)

        ax.cla()
        ax.set_title("YOLO Object Angular Spans (edge-to-edge)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.set_ylim(-HFOV_DEG/2.0 - 5, HFOV_DEG/2.0 + 5)
        ax.grid(True)

        xs_latest = None

        for ts, dets in snap_copy:
            x = ts - t0
            xs_latest = x

            for d in dets:
                lw = 1.0 + 6.0 * d["conf"]  # thickness by confidence
                ax.plot([x, x], [d["a_min"], d["a_max"]], linewidth=lw, color="blue")

        if xs_latest is not None:
            ax.set_xlim(max(0, xs_latest - PLOT_WINDOW_SEC), xs_latest + 0.5)
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
        frame = frame[:, :, :3]

        # If detections look strange, try commenting this line:
        frame = frame[:, :, ::-1]

        with frame_lock:
            latest_frame = frame

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    yolo_thread.join(timeout=1.0)
    picam2.stop()
