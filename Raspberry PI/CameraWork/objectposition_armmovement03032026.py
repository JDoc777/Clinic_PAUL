"""
Qt + PyQtGraph test (Pi / Picamera2):
YOLO -> bbox center (cx,cy) -> delta to camera center (dx,dy)

- Qt window with:
  - LEFT: PyQtGraph plot of dx, dy vs time (pixels)
  - RIGHT: small video preview with boxes, crosshair, and dx/dy text
- Ctrl+C works (Qt event loop won't swallow SIGINT)
- Close window OR Ctrl+C to exit cleanly

Install:
  sudo apt update
  sudo apt install -y python3-pyqtgraph python3-pyqt5
  pip install ultralytics opencv-python

Run:
  python3 qt_dxdy_test_picam2.py
"""

import os
# os.environ["PYQTGRAPH_NO_OPENGL"] = "1"  # uncomment if OpenGL causes issues

import sys
import time
import threading
import signal
from collections import deque

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from picamera2 import Picamera2

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


# ---------------- SETTINGS ----------------
CAMERA_SIZE = (1920, 1080)   # (W, H)
MODEL_PATH = "yolov8n.pt"
YOLO_SIZE = 640
CONF_THRES = 0.30

# How often to print
PRINT_HZ = 8

# Preview panel size (smaller)
PREVIEW_W = 480
VIDEO_UPDATE_HZ = 30
DRAW_YOLO_BOXES_ON_VIDEO = True

# Plot settings
PLOT_WINDOW_SEC = 10.0
PLOT_UPDATE_HZ = 25

# Which detection to report:
#  - "best_conf" : highest confidence
#  - "largest"   : biggest bbox area
PICK_MODE = "best_conf"

torch.set_num_threads(1)

# ---------------- Shared state ----------------
stop_event = threading.Event()

frame_lock = threading.Lock()
latest_frame_rgb = None   # full-res RGB

det_lock = threading.Lock()
latest_dets = []          # list of dicts {"x1","y1","x2","y2","label","conf","cx","cy"}

err_lock = threading.Lock()
latest_err = None         # dict {"t","cx","cy","dx","dy","label","conf"}


# ---------------- Helpers ----------------
def pick_detection(dets, mode="best_conf"):
    if not dets:
        return None
    if mode == "largest":
        return max(dets, key=lambda d: d["area"])
    return max(dets, key=lambda d: d["conf"])


def bgr_for_label(label: str):
    h = abs(hash(label)) % 180
    hsv = np.array([[[h, 220, 255]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))


# ---------------- Threads ----------------
def camera_worker(picam2: Picamera2):
    global latest_frame_rgb
    while not stop_event.is_set():
        frame = picam2.capture_array()[:, :, :3]  # RGB
        with frame_lock:
            latest_frame_rgb = frame


def yolo_worker(model: YOLO):
    global latest_dets, latest_err

    last_print = 0.0

    while not stop_event.is_set():
        with frame_lock:
            frame = None if latest_frame_rgb is None else latest_frame_rgb.copy()

        if frame is None:
            time.sleep(0.005)
            continue

        h, w = frame.shape[:2]
        aim_x, aim_y = w / 2.0, h / 2.0

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        results = model(frame_bgr, imgsz=YOLO_SIZE, verbose=False)
        boxes = results[0].boxes

        dets = []
        if boxes is not None and len(boxes) > 0:
            for b in boxes:
                conf = float(b.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(b.cls[0])
                label = model.names[cls_id]

                x1, y1, x2, y2 = b.xyxy[0].tolist()
                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)
                area = max(0.0, (x2 - x1)) * max(0.0, (y2 - y1))

                dets.append({
                    "label": label, "conf": conf,
                    "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
                    "cx": float(cx), "cy": float(cy),
                    "area": float(area),
                })

        chosen = pick_detection(dets, PICK_MODE)
        now = time.time()

        with det_lock:
            latest_dets = dets

        if chosen is not None:
            dx = chosen["cx"] - aim_x  # + right
            dy = chosen["cy"] - aim_y  # + down

            with err_lock:
                latest_err = {
                    "t": now,
                    "label": chosen["label"],
                    "conf": chosen["conf"],
                    "cx": chosen["cx"],
                    "cy": chosen["cy"],
                    "dx": dx,
                    "dy": dy,
                    "aim_x": aim_x,
                    "aim_y": aim_y,
                    "w": w,
                    "h": h,
                }
        else:
            with err_lock:
                latest_err = None

        if now - last_print >= (1.0 / PRINT_HZ):
            last_print = now
            if chosen is None:
                print("No detections above threshold.")
            else:
                print(
                    f"{chosen['label']} (conf={chosen['conf']:.2f}) | "
                    f"center=({chosen['cx']:.1f},{chosen['cy']:.1f}) px | "
                    f"dx={dx:.1f} px, dy={dy:.1f} px"
                )


# ---------------- Qt GUI ----------------
class DxDyWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("YOLO center error: dx/dy (Qt + PyQtGraph)")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ---- Plot (left) ----
        self.plot = pg.PlotWidget()
        layout.addWidget(self.plot, stretch=2)

        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.setLabel("left", "Error", units="px")
        self.plot.addLegend()

        self.dx_curve = self.plot.plot([], [], pen=pg.mkPen(width=2), name="dx (+ right)")
        self.dy_curve = self.plot.plot([], [], pen=pg.mkPen(width=2, style=QtCore.Qt.DashLine), name="dy (+ down)")

        # ---- Video (right) ----
        self.video_label = QtWidgets.QLabel()
        self.video_label.setMinimumWidth(PREVIEW_W)
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setText("Video preview")
        layout.addWidget(self.video_label, stretch=1)

        # history buffers
        self.t0 = time.time()
        self.hist_t = deque()
        self.hist_dx = deque()
        self.hist_dy = deque()

        self.last_vid = 0.0

        # timers
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(1000 / PLOT_UPDATE_HZ))

    def closeEvent(self, event):
        stop_event.set()
        event.accept()

    def update_ui(self):
        self.update_plot()
        self.update_video()

    def update_plot(self):
        # pull latest error snapshot
        with err_lock:
            e = None if latest_err is None else dict(latest_err)

        if e is None:
            # still keep plot scrolling? no—just leave last values
            return

        t = e["t"] - self.t0
        self.hist_t.append(t)
        self.hist_dx.append(e["dx"])
        self.hist_dy.append(e["dy"])

        # trim
        while self.hist_t and (t - self.hist_t[0]) > PLOT_WINDOW_SEC:
            self.hist_t.popleft()
            self.hist_dx.popleft()
            self.hist_dy.popleft()

        xs = list(self.hist_t)
        dxs = list(self.hist_dx)
        dys = list(self.hist_dy)

        self.dx_curve.setData(xs, dxs)
        self.dy_curve.setData(xs, dys)

        # x-range
        self.plot.setXRange(max(0.0, t - PLOT_WINDOW_SEC), t + 0.1, padding=0.0)

        # y autoscale with padding
        ys = dxs + dys
        if ys:
            y_min = min(ys) - 30
            y_max = max(ys) + 30
            if y_min == y_max:
                y_min -= 1
                y_max += 1
            self.plot.setYRange(y_min, y_max)

    def update_video(self):
        now = time.time()
        if now - self.last_vid < (1.0 / VIDEO_UPDATE_HZ):
            return
        self.last_vid = now

        with frame_lock:
            frame = None if latest_frame_rgb is None else latest_frame_rgb.copy()
        if frame is None:
            return

        h, w = frame.shape[:2]

        # resize to preview width
        new_w = PREVIEW_W
        new_h = int(new_w * h / w)
        frame_small = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)  # still RGB

        # draw overlays
        # aim point
        ax = int(new_w / 2)
        ay = int(new_h / 2)
        cv2.drawMarker(
            frame_small, (ax, ay), (255, 255, 0),
            markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2
        )

        if DRAW_YOLO_BOXES_ON_VIDEO:
            with det_lock:
                dets = list(latest_dets)

            sx = new_w / w
            sy = new_h / h

            for d in dets:
                x1 = int(d["x1"] * sx)
                y1 = int(d["y1"] * sy)
                x2 = int(d["x2"] * sx)
                y2 = int(d["y2"] * sy)
                cx = int(d["cx"] * sx)
                cy = int(d["cy"] * sy)

                label = d["label"]
                conf = d["conf"]

                bgr = bgr_for_label(label)
                rgb = (bgr[2], bgr[1], bgr[0])

                cv2.rectangle(frame_small, (x1, y1), (x2, y2), rgb, 2)
                cv2.circle(frame_small, (cx, cy), 4, rgb, -1)
                cv2.putText(
                    frame_small, f"{label} {conf:.2f}", (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, rgb, 1, cv2.LINE_AA
                )

        # dx/dy text (chosen only)
        with err_lock:
            e = None if latest_err is None else dict(latest_err)
        if e is not None:
            # scale center to preview coords
            cxp = int(e["cx"] * (new_w / w))
            cyp = int(e["cy"] * (new_h / h))
            cv2.line(frame_small, (ax, ay), (cxp, cyp), (0, 0, 255), 2)
            cv2.putText(
                frame_small, f"dx={e['dx']:.0f}px  dy={e['dy']:.0f}px",
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA
            )

        # RGB numpy -> QImage
        qimg = QtGui.QImage(
            frame_small.data,
            frame_small.shape[1],
            frame_small.shape[0],
            frame_small.strides[0],
            QtGui.QImage.Format_RGB888,
        )
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))


# ---------------- Ctrl+C handling for Qt ----------------
def _handle_sigint(sig, frame):
    stop_event.set()
    raise KeyboardInterrupt


def main():
    print("Loading YOLO...")
    model = YOLO(MODEL_PATH)
    print("Model loaded.")

    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
    picam2.configure(cfg)
    picam2.start()
    time.sleep(0.8)

    cam_thread = threading.Thread(target=camera_worker, args=(picam2,), daemon=True)
    det_thread = threading.Thread(target=yolo_worker, args=(model,), daemon=True)
    cam_thread.start()
    det_thread.start()

    app = QtWidgets.QApplication(sys.argv)

    # --- make Ctrl+C work with Qt event loop ---
    signal.signal(signal.SIGINT, _handle_sigint)
    _interrupt_timer = QtCore.QTimer()
    _interrupt_timer.timeout.connect(lambda: None)
    _interrupt_timer.start(250)
    # ------------------------------------------

    win = DxDyWindow()
    win.resize(1200, 650)
    win.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            picam2.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()