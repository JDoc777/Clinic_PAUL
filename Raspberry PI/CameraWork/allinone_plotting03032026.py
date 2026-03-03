"""
ALL-IN-ONE (Pi / Picamera2 / YOLO / Qt + PyQtGraph)

UPDATED per request:
- Drop the dx/dy (pixel error) plot entirely
- Keep ONLY the two plots:
    1) Horizontal (yaw) angular span vs time
    2) Vertical (pitch) angular span vs time (NOT upside down)
- Clean terminal printing:
    A) Print all active tracked objects with BOTH spans (yaw + pitch)
    B) Then print the MOST CONFIDENT object again, plus dx/dy pixels to center

Quit:
- Close window OR Ctrl+C (camera released cleanly)
"""

import os
# os.environ["PYQTGRAPH_NO_OPENGL"] = "1"  # uncomment if OpenGL causes issues

import sys
import time
import math
import threading
import signal
from collections import deque

import numpy as np
import cv2
import torch
from ultralytics import YOLO
from picamera2 import Picamera2

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


# ---------------- SETTINGS ----------------
CAMERA_SIZE = (1920, 1080)  # (W, H)

MODEL_PATH = "yolov8n.pt"
YOLO_SIZE = 640
CONF_THRES = 0.30

HFOV_DEG = 102.0
USE_ATAN_MAPPING = True

TRACK_TTL_SEC = 1.2
MATCH_MAX_DEG = 8.0     # 2D angular match threshold (deg)
PRINT_HZ = 8

# Plots
PLOT_WINDOW_SEC = 10.0
PLOT_UPDATE_HZ = 25
SHOW_TEXT_LABELS = True

# Video preview
SHOW_VIDEO = True
VIDEO_W = 640
VIDEO_UPDATE_HZ = 30
DRAW_YOLO_BOXES_ON_VIDEO = True

torch.set_num_threads(1)


# ---------------- Derived FOV ----------------
IMG_W, IMG_H = CAMERA_SIZE
VFOV_DEG = math.degrees(
    2.0 * math.atan(math.tan(math.radians(HFOV_DEG / 2.0)) * (IMG_H / IMG_W))
)
print(f"[INFO] HFOV={HFOV_DEG:.2f}°, derived VFOV={VFOV_DEG:.2f}° for {IMG_W}x{IMG_H}")


# ---------------- Shared state ----------------
stop_event = threading.Event()

frame_lock = threading.Lock()
latest_frame = None  # RGB full-res

tracks_lock = threading.Lock()
latest_tracks = []  # list of dicts for plotting (tracked, smoothed-ish spans)

det_lock = threading.Lock()
latest_dets = []  # list of dicts for video overlay

best_lock = threading.Lock()
best_obj = None   # dict: best detection snapshot for printing + dx/dy overlay


# ---------------- Ctrl+C handling for Qt ----------------
def _handle_sigint(sig, frame):
    stop_event.set()
    raise KeyboardInterrupt


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


# Vertical (FIXED sign): top of image -> +angle, bottom -> -angle
def pixel_y_to_angle_deg_linear(y, img_h, vfov_deg):
    y_norm = (y - (img_h / 2.0)) / (img_h / 2.0)
    return -y_norm * (vfov_deg / 2.0)

def pixel_y_to_angle_deg_atan(y, img_h, vfov_deg):
    fy = (img_h / 2.0) / math.tan(math.radians(vfov_deg / 2.0))
    return -math.degrees(math.atan2((y - (img_h / 2.0)), fy))

def pixel_y_to_angle_deg(y, img_h, vfov_deg):
    if USE_ATAN_MAPPING:
        return pixel_y_to_angle_deg_atan(y, img_h, vfov_deg)
    return pixel_y_to_angle_deg_linear(y, img_h, vfov_deg)


# ---------------- Color helpers ----------------
def bgr_for_label(label: str):
    h = abs(hash(label)) % 180
    hsv = np.array([[[h, 220, 255]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

label_color = {}
def qcolor_for_label(label: str):
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
    global latest_tracks, latest_dets, best_obj

    img_w, img_h = CAMERA_SIZE
    aim_x = img_w / 2.0
    aim_y = img_h / 2.0

    tracks = {}      # tid -> track dict
    next_id = 1
    last_print = 0.0

    while not stop_event.is_set():
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is None:
            time.sleep(0.005)
            continue

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        results = model(frame_bgr, imgsz=YOLO_SIZE, verbose=False)

        boxes = results[0].boxes
        now = time.time()

        detections = []
        dets_for_video = []

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)

                dets_for_video.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "label": label, "conf": conf})

                cx = 0.5 * (x1 + x2)
                cy = 0.5 * (y1 + y2)

                # yaw span
                yaw_left  = pixel_x_to_angle_deg(x1, img_w, HFOV_DEG)
                yaw_right = pixel_x_to_angle_deg(x2, img_w, HFOV_DEG)
                yaw_min = min(yaw_left, yaw_right)
                yaw_max = max(yaw_left, yaw_right)
                yaw_ctr = 0.5 * (yaw_min + yaw_max)

                # pitch span
                pit_top = pixel_y_to_angle_deg(y1, img_h, VFOV_DEG)
                pit_bot = pixel_y_to_angle_deg(y2, img_h, VFOV_DEG)
                pit_min = min(pit_top, pit_bot)
                pit_max = max(pit_top, pit_bot)
                pit_ctr = 0.5 * (pit_min + pit_max)

                detections.append({
                    "label": label,
                    "conf": conf,
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "cx": cx, "cy": cy,
                    "yaw_min": yaw_min, "yaw_max": yaw_max, "yaw_ctr": yaw_ctr,
                    "pit_min": pit_min, "pit_max": pit_max, "pit_ctr": pit_ctr,
                })

        # publish dets for video
        with det_lock:
            latest_dets = dets_for_video

        # choose best-conf object for printing + dx/dy
        if detections:
            best = max(detections, key=lambda d: d["conf"])
            dx = best["cx"] - aim_x  # + right
            dy = best["cy"] - aim_y  # + down
            best_snapshot = dict(best)
            best_snapshot["dx"] = dx
            best_snapshot["dy"] = dy
        else:
            best_snapshot = None

        with best_lock:
            best_obj = best_snapshot

        # ---- match detections to existing tracks (2D angular center distance, same label) ----
        matched_track_ids = set()

        for det in detections:
            best_tid = None
            best_err = None

            for tid, tr in tracks.items():
                if tr["label"] != det["label"]:
                    continue
                if tid in matched_track_ids:
                    continue

                dyaw = det["yaw_ctr"] - tr["yaw_ctr"]
                dpit = det["pit_ctr"] - tr["pit_ctr"]
                err = math.hypot(dyaw, dpit)

                if err <= MATCH_MAX_DEG and (best_err is None or err < best_err):
                    best_err = err
                    best_tid = tid

            if best_tid is None:
                tid = next_id
                next_id += 1
                tracks[tid] = {
                    "id": tid,
                    "label": det["label"],
                    "conf": det["conf"],
                    "last_seen": now,
                    "yaw_min": det["yaw_min"], "yaw_max": det["yaw_max"], "yaw_ctr": det["yaw_ctr"],
                    "pit_min": det["pit_min"], "pit_max": det["pit_max"], "pit_ctr": det["pit_ctr"],
                }
                matched_track_ids.add(tid)
            else:
                tr = tracks[best_tid]
                tr["conf"] = det["conf"]
                tr["last_seen"] = now
                tr["yaw_min"] = det["yaw_min"]
                tr["yaw_max"] = det["yaw_max"]
                tr["yaw_ctr"] = det["yaw_ctr"]
                tr["pit_min"] = det["pit_min"]
                tr["pit_max"] = det["pit_max"]
                tr["pit_ctr"] = det["pit_ctr"]
                matched_track_ids.add(best_tid)

        # prune old tracks
        dead = [tid for tid, tr in tracks.items() if (now - tr["last_seen"]) > TRACK_TTL_SEC]
        for tid in dead:
            del tracks[tid]

        with tracks_lock:
            latest_tracks = [dict(t) for t in tracks.values()]

        # ---------------- CLEAN PRINT ----------------
        if now - last_print >= (1.0 / PRINT_HZ):
            last_print = now

            tr_list = sorted(tracks.values(), key=lambda x: x["id"])

            if not tr_list:
                print("Tracks: (none)")
            else:
                # One-line summary of all tracks
                parts = []
                for t in tr_list:
                    parts.append(
                        f"{t['label']}#{t['id']} "
                        f"yaw[{t['yaw_min']:+.1f}..{t['yaw_max']:+.1f}]° "
                        f"pit[{t['pit_min']:+.1f}..{t['pit_max']:+.1f}]° "
                        f"c={t['conf']:.2f}"
                    )
                print("Tracks: " + " | ".join(parts))

            # Best object (raw detection) + dx/dy
            if best_snapshot is None:
                print("Best: (none)")
            else:
                print(
                    "Best: "
                    f"{best_snapshot['label']} "
                    f"yaw[{best_snapshot['yaw_min']:+.1f}..{best_snapshot['yaw_max']:+.1f}]° "
                    f"pit[{best_snapshot['pit_min']:+.1f}..{best_snapshot['pit_max']:+.1f}]° "
                    f"c={best_snapshot['conf']:.2f} | "
                    f"dx={best_snapshot['dx']:+.1f}px dy={best_snapshot['dy']:+.1f}px"
                )


# ---------------- PyQtGraph GUI ----------------
class SpanWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Yaw + Pitch spans + Video (Qt + PyQtGraph)")

        self.t0 = time.time()
        self.snapshots = deque()  # (timestamp, tracks_list)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        grid = QtWidgets.QGridLayout(central)

        # ---- Yaw plot ----
        self.plot_yaw = pg.PlotWidget()
        self.plot_yaw.showGrid(x=True, y=True, alpha=0.3)
        self.plot_yaw.setLabel("bottom", "Time", units="s")
        self.plot_yaw.setLabel("left", "Yaw angle", units="deg")
        self.plot_yaw.setYRange(-HFOV_DEG / 2.0 - 5, HFOV_DEG / 2.0 + 5)
        grid.addWidget(self.plot_yaw, 0, 0)

        # ---- Pitch plot ----
        self.plot_pit = pg.PlotWidget()
        self.plot_pit.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pit.setLabel("bottom", "Time", units="s")
        self.plot_pit.setLabel("left", "Pitch angle", units="deg")
        self.plot_pit.setYRange(-VFOV_DEG / 2.0 - 5, VFOV_DEG / 2.0 + 5)
        grid.addWidget(self.plot_pit, 1, 0)

        # ---- Video ----
        self.video_label = QtWidgets.QLabel()
        self.video_label.setMinimumWidth(VIDEO_W)
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setText("Video preview")
        grid.addWidget(self.video_label, 0, 1, 2, 1)

        self.last_vid = 0.0

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(1000 / PLOT_UPDATE_HZ))

    def closeEvent(self, event):
        stop_event.set()
        event.accept()

    def update_ui(self):
        self.update_plots()
        if SHOW_VIDEO:
            self.update_video()

    def update_plots(self):
        now = time.time()

        with tracks_lock:
            tr_copy = [dict(t) for t in latest_tracks]

        self.snapshots.append((now, tr_copy))
        while self.snapshots and (now - self.snapshots[0][0]) > PLOT_WINDOW_SEC:
            self.snapshots.popleft()

        self.plot_yaw.clear()
        self.plot_pit.clear()

        for ts, track_list in self.snapshots:
            x = ts - self.t0
            for tr in track_list:
                c = qcolor_for_label(tr["label"])
                w = 1 + int(6 * float(tr.get("conf", 0.5)))
                pen = pg.mkPen(c, width=w)

                self.plot_yaw.plot([x, x], [tr["yaw_min"], tr["yaw_max"]], pen=pen)
                self.plot_pit.plot([x, x], [tr["pit_min"], tr["pit_max"]], pen=pen)

        if SHOW_TEXT_LABELS and self.snapshots:
            ts_last, tracks_last = self.snapshots[-1]
            x_last = ts_last - self.t0

            for tr in tracks_last:
                c = qcolor_for_label(tr["label"])

                ymid_yaw = 0.5 * (tr["yaw_min"] + tr["yaw_max"])
                txt_yaw = pg.TextItem(f'{tr["label"]}#{tr["id"]}', anchor=(0, 0.5))
                txt_yaw.setColor(c)
                txt_yaw.setPos(x_last + 0.05, ymid_yaw)
                self.plot_yaw.addItem(txt_yaw)

                ymid_pit = 0.5 * (tr["pit_min"] + tr["pit_max"])
                txt_pit = pg.TextItem(f'{tr["label"]}#{tr["id"]}', anchor=(0, 0.5))
                txt_pit.setColor(c)
                txt_pit.setPos(x_last + 0.05, ymid_pit)
                self.plot_pit.addItem(txt_pit)

            self.plot_yaw.setXRange(max(0, x_last - PLOT_WINDOW_SEC), x_last + 1.0, padding=0.0)
            self.plot_pit.setXRange(max(0, x_last - PLOT_WINDOW_SEC), x_last + 1.0, padding=0.0)

    def update_video(self):
        now = time.time()
        if now - self.last_vid < (1.0 / VIDEO_UPDATE_HZ):
            return
        self.last_vid = now

        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()
        if frame is None:
            return

        h, w = frame.shape[:2]
        new_w = VIDEO_W
        new_h = int(new_w * h / w)
        frame_small = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)  # RGB

        # crosshair at image center
        ax = int(new_w / 2)
        ay = int(new_h / 2)
        cv2.drawMarker(frame_small, (ax, ay), (255, 255, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

        # draw boxes
        if DRAW_YOLO_BOXES_ON_VIDEO:
            with det_lock:
                dets_copy = list(latest_dets)

            sx = new_w / w
            sy = new_h / h

            for d in dets_copy:
                x1 = int(d["x1"] * sx)
                y1 = int(d["y1"] * sy)
                x2 = int(d["x2"] * sx)
                y2 = int(d["y2"] * sy)
                label = d["label"]
                conf = d["conf"]

                bgr = bgr_for_label(label)
                rgb = (bgr[2], bgr[1], bgr[0])

                cv2.rectangle(frame_small, (x1, y1), (x2, y2), rgb, 2)
                txt = f"{label} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                y_text = max(0, y1 - th - 6)
                cv2.rectangle(frame_small, (x1, y_text), (x1 + tw + 6, y_text + th + 6), rgb, -1)
                cv2.putText(frame_small, txt, (x1 + 3, y_text + th + 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # best object overlay: dx/dy line + text
        with best_lock:
            b = None if best_obj is None else dict(best_obj)

        if b is not None:
            cxp = int(b["cx"] * (new_w / w))
            cyp = int(b["cy"] * (new_h / h))
            cv2.line(frame_small, (ax, ay), (cxp, cyp), (0, 0, 255), 2)
            cv2.putText(
                frame_small, f"BEST: {b['label']} c={b['conf']:.2f}  dx={b['dx']:+.0f}px dy={b['dy']:+.0f}px",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA
            )

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

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.8)

    cam_thread = threading.Thread(target=camera_worker, args=(picam2,), daemon=True)
    det_thread = threading.Thread(target=yolo_worker, args=(model,), daemon=True)
    cam_thread.start()
    det_thread.start()

    app = QtWidgets.QApplication(sys.argv)

    # Ctrl+C fix (Qt event loop)
    signal.signal(signal.SIGINT, _handle_sigint)
    _interrupt_timer = QtCore.QTimer()
    _interrupt_timer.timeout.connect(lambda: None)
    _interrupt_timer.start(250)

    win = SpanWindow()
    win.resize(1400, 800)
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