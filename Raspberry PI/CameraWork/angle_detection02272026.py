"""
PyQtGraph live angular-span plot + optional Qt video preview (no Matplotlib, no cv2.imshow)

- Captures frames from Picamera2 (RGB)
- YOLO runs in a worker thread (converts RGB -> BGR before inference)
- Tracks objects by angular center and label
- PyQtGraph window shows live “vertical span” lines over time
- Optional video preview pane (Qt QLabel) showing the camera feed
- Optional YOLO boxes drawn on the preview
- Terminal prints compact track snapshots

Install (recommended on Pi via apt):
  sudo apt update
  sudo apt install -y python3-pyqtgraph python3-pyqt5 python3-opengl libgl1-mesa-dri libgl1-mesa-glx

If you want to avoid OpenGL entirely, uncomment PYQTGRAPH_NO_OPENGL env var below.

Run:
  python3 your_script.py
Close the window to stop.
"""

import os
# Uncomment to prevent PyQtGraph from trying to use OpenGL (often safer on Pi)
# os.environ["PYQTGRAPH_NO_OPENGL"] = "1"

import sys
import time
import math
import threading
from collections import deque

import numpy as np
import cv2

from picamera2 import Picamera2
from ultralytics import YOLO
import torch

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


# ---------------- SETTINGS ----------------
CAMERA_SIZE = (1920, 1080)

MODEL_PATH = "yolov8n.pt"
YOLO_SIZE = 640            # try 768/960/1280 if you truly don't care about speed
CONF_THRES = 0.30          # lower helps far objects; raise to reduce junk

HFOV_DEG = 102.0

TRACK_TTL_SEC = 1.2
MATCH_MAX_DEG = 8.0
PRINT_HZ = 8

# PyQtGraph plot
PLOT_WINDOW_SEC = 10.0     # scrolling window length
PLOT_UPDATE_HZ = 30        # UI refresh rate (smooth)
SHOW_TEXT_LABELS = True    # label each active track at the right edge

# Angle mapping
USE_ATAN_MAPPING = True

# Video preview (Qt)
SHOW_VIDEO = True
VIDEO_W = 640
VIDEO_UPDATE_HZ = 30
DRAW_YOLO_BOXES_ON_VIDEO = True

torch.set_num_threads(1)

# ---------------- Shared state ----------------
frame_lock = threading.Lock()
latest_frame = None  # RGB

tracks_lock = threading.Lock()
latest_tracks = []  # list of dicts: {"id","label","a_min","a_max","a_ctr","conf"}

det_lock = threading.Lock()
latest_dets = []  # list of dicts: {"x1","y1","x2","y2","label","conf"} in ORIGINAL frame coords

stop_event = threading.Event()


# ---------------- Angle helpers ----------------
def pixel_x_to_angle_deg_linear(x, img_w, hfov_deg):
    x_norm = (x - (img_w / 2.0)) / (img_w / 2.0)
    return x_norm * (hfov_deg / 2.0)

def pixel_x_to_angle_deg_atan(x, img_w, hfov_deg):
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    return math.degrees(math.atan2((x - (img_w / 2.0)), fx))

def pixel_x_to_angle_deg(x, img_w, hfov_deg):
    if USE_ATAN_MAPPING:
        return pixel_x_to_angle_deg_atan(x, img_w, hfov_deg)
    return pixel_x_to_angle_deg_linear(x, img_w, hfov_deg)


# ---------------- Color helpers ----------------
def bgr_for_label(label: str):
    # deterministic-ish color from hash -> HSV -> BGR
    h = abs(hash(label)) % 180  # OpenCV H range is 0..179
    hsv = np.array([[[h, 220, 255]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

label_color = {}
def qcolor_for_label(label: str):
    # deterministic-ish HSV color from label hash
    if label not in label_color:
        h = (abs(hash(label)) % 360) / 360.0
        label_color[label] = pg.hsvColor(h, sat=0.9, val=0.95, alpha=1.0)
    return label_color[label]


# ---------------- Camera thread ----------------
def camera_worker(picam2: Picamera2):
    global latest_frame
    while not stop_event.is_set():
        frame = picam2.capture_array()
        frame = frame[:, :, :3]  # RGB
        with frame_lock:
            latest_frame = frame


# ---------------- YOLO + tracking thread ----------------
def yolo_worker(model: YOLO):
    global latest_tracks, latest_dets

    img_w = CAMERA_SIZE[0]

    tracks = {}
    next_id = 1
    last_print = 0.0

    while not stop_event.is_set():
        # Grab current frame
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            time.sleep(0.005)
            continue

        # --- YOUR INFERENCE SNIPPET ---
        # Convert RGB -> BGR for YOLO
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = model(frame_bgr, imgsz=YOLO_SIZE, verbose=False)
        # -----------------------------

        boxes = results[0].boxes
        now = time.time()

        detections_for_tracking = []
        dets_for_video = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Save for video drawing
                dets_for_video.append({
                    "x1": float(x1), "y1": float(y1),
                    "x2": float(x2), "y2": float(y2),
                    "label": label, "conf": conf
                })

                # Convert to angular span for tracking
                a_left  = pixel_x_to_angle_deg(x1, img_w, HFOV_DEG)
                a_right = pixel_x_to_angle_deg(x2, img_w, HFOV_DEG)
                a_min = min(a_left, a_right)
                a_max = max(a_left, a_right)
                a_ctr = 0.5 * (a_min + a_max)

                detections_for_tracking.append({
                    "label": label,
                    "conf": conf,
                    "a_min": a_min,
                    "a_max": a_max,
                    "a_ctr": a_ctr
                })

        # Publish latest dets
        with det_lock:
            latest_dets = dets_for_video

        # ---- match detections to existing tracks (nearest center angle, same label) ----
        matched_track_ids = set()

        for det in detections_for_tracking:
            best_tid = None
            best_err = None

            for tid, tr in tracks.items():
                if tr["label"] != det["label"]:
                    continue
                if tid in matched_track_ids:
                    continue

                err = abs(det["a_ctr"] - tr["a_ctr"])
                if err <= MATCH_MAX_DEG and (best_err is None or err < best_err):
                    best_err = err
                    best_tid = tid

            if best_tid is None:
                tid = next_id
                next_id += 1
                tracks[tid] = {
                    "id": tid,
                    "label": det["label"],
                    "a_min": det["a_min"],
                    "a_max": det["a_max"],
                    "a_ctr": det["a_ctr"],
                    "conf": det["conf"],
                    "last_seen": now
                }
                matched_track_ids.add(tid)
            else:
                tr = tracks[best_tid]
                tr["a_min"] = det["a_min"]
                tr["a_max"] = det["a_max"]
                tr["a_ctr"] = det["a_ctr"]
                tr["conf"] = det["conf"]
                tr["last_seen"] = now
                matched_track_ids.add(best_tid)

        # ---- prune old tracks ----
        dead = [tid for tid, tr in tracks.items() if (now - tr["last_seen"]) > TRACK_TTL_SEC]
        for tid in dead:
            del tracks[tid]

        # Publish latest tracks for the UI thread
        track_list = list(tracks.values())
        with tracks_lock:
            latest_tracks = [dict(t) for t in track_list]

        # Terminal print
        if now - last_print >= (1.0 / PRINT_HZ):
            last_print = now
            if track_list:
                s = " | ".join(
                    [f"{t['label']}#{t['id']} [{t['a_min']:.1f}..{t['a_max']:.1f}]° (c={t['conf']:.2f})"
                     for t in sorted(track_list, key=lambda x: x["id"])]
                )
                print(s)


# ---------------- PyQtGraph GUI ----------------
class TrackPlotWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tracked Angular Spans + Video (PyQtGraph)")

        self.t0 = time.time()
        self.snapshots = deque()  # (timestamp, tracks_list)

        # --- layout: plot + video ---
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Plot
        self.plot = pg.PlotWidget()
        layout.addWidget(self.plot, stretch=2)

        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Angle", units="deg")
        self.plot.setYRange(-HFOV_DEG/2.0 - 5, HFOV_DEG/2.0 + 5)

        # Video
        self.video_label = QtWidgets.QLabel()
        self.video_label.setMinimumWidth(VIDEO_W)
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setText("Video preview")
        layout.addWidget(self.video_label, stretch=1)

        self.last_vid = 0.0

        # Update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(1000 / PLOT_UPDATE_HZ))

    def closeEvent(self, event):
        stop_event.set()
        event.accept()

    def update_ui(self):
        self.update_plot()
        if SHOW_VIDEO:
            self.update_video()

    def update_plot(self):
        now = time.time()

        # Pull latest tracks (even if YOLO updates slower)
        with tracks_lock:
            tr_copy = [dict(t) for t in latest_tracks]

        # Add to rolling buffer at UI rate so x-axis scrolls smoothly
        self.snapshots.append((now, tr_copy))
        while self.snapshots and (now - self.snapshots[0][0]) > PLOT_WINDOW_SEC:
            self.snapshots.popleft()

        # Redraw
        self.plot.clear()

        xs_latest = None
        for ts, track_list in self.snapshots:
            x = ts - self.t0
            xs_latest = x
            for tr in track_list:
                c = qcolor_for_label(tr["label"])
                w = 1 + int(6 * float(tr.get("conf", 0.5)))
                pen = pg.mkPen(c, width=w)
                self.plot.plot([x, x], [tr["a_min"], tr["a_max"]], pen=pen)

        # Add labels at the newest time slice only
        if SHOW_TEXT_LABELS and self.snapshots:
            ts_last, tracks_last = self.snapshots[-1]
            x_last = ts_last - self.t0
            for tr in tracks_last:
                y_mid = 0.5 * (tr["a_min"] + tr["a_max"])
                txt = pg.TextItem(f'{tr["label"]}#{tr["id"]}', anchor=(0, 0.5))
                txt.setColor(qcolor_for_label(tr["label"]))
                txt.setPos(x_last + 0.05, y_mid)
                self.plot.addItem(txt)

            self.plot.setXRange(max(0, x_last - PLOT_WINDOW_SEC), x_last + 1.0, padding=0.0)

    def update_video(self):
        now = time.time()
        if now - self.last_vid < (1.0 / VIDEO_UPDATE_HZ):
            return
        self.last_vid = now

        # Grab latest frame (RGB)
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()
        if frame is None:
            return

        h, w = frame.shape[:2]
        new_w = VIDEO_W
        new_h = int(new_w * h / w)
        frame_small = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

        if DRAW_YOLO_BOXES_ON_VIDEO:
            # pull dets (original frame coords), then scale to preview
            with det_lock:
                dets_copy = list(latest_dets)

            sx = new_w / w
            sy = new_h / h

            # draw on RGB image (OpenCV expects BGR for drawing colors, but we can still draw;
            # just convert our chosen color to RGB-ish by reversing)
            for d in dets_copy:
                x1 = int(d["x1"] * sx)
                y1 = int(d["y1"] * sy)
                x2 = int(d["x2"] * sx)
                y2 = int(d["y2"] * sy)
                label = d["label"]
                conf = d["conf"]

                bgr = bgr_for_label(label)
                rgb = (bgr[2], bgr[1], bgr[0])  # convert BGR->RGB for drawing on RGB buffer

                cv2.rectangle(frame_small, (x1, y1), (x2, y2), rgb, 2)
                txt = f"{label} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                y_text = max(0, y1 - th - 6)
                cv2.rectangle(frame_small, (x1, y_text), (x1 + tw + 6, y_text + th + 6), rgb, -1)
                cv2.putText(frame_small, txt, (x1 + 3, y_text + th + 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # Convert RGB numpy -> QImage
        qimg = QtGui.QImage(
            frame_small.data,
            frame_small.shape[1],
            frame_small.shape[0],
            frame_small.strides[0],
            QtGui.QImage.Format_RGB888,
        )
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))


def main():
    print("Loading YOLO...")
    model = YOLO(MODEL_PATH)
    print("Model loaded.")

    # Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
    picam2.configure(config)
    picam2.start()

    # Let auto-exposure settle a bit
    time.sleep(0.8)

    # Threads
    cam_thread = threading.Thread(target=camera_worker, args=(picam2,), daemon=True)
    yolo_thread = threading.Thread(target=yolo_worker, args=(model,), daemon=True)
    cam_thread.start()
    yolo_thread.start()

    # Qt UI (must be main thread)
    app = QtWidgets.QApplication(sys.argv)
    win = TrackPlotWindow()
    win.resize(1200, 650)
    win.show()

    try:
        sys.exit(app.exec_())
    finally:
        stop_event.set()
        # best-effort cleanup
        try:
            picam2.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()