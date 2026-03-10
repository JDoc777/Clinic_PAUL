"""
VisionProcessor (Picamera2 + YOLO + Tracking + Qt/PyQtGraph Window)

Designed to integrate into run_all.py which already has:
    app = QApplication(sys.argv)
    pg_live_plot_loop(...)  # likely runs the Qt event loop

KEY BEHAVIOR:
- Starts camera + YOLO threads immediately (non-blocking)
- Provides create_window() which returns a Qt window you can show/hide
- Window updates via QTimer (works under your existing Qt loop)
- Individual UI toggles for:
    - yaw plot
    - pitch plot
    - video preview
    - labels
    - YOLO boxes overlay
    - BEST overlay

Factory:
    create_and_run(shared_data, ...) -> VisionProcessor
"""

from __future__ import annotations

import time
import math
import threading
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import cv2
import torch
from ultralytics import YOLO
from picamera2 import Picamera2

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


# ---------------- defaults ----------------
DEFAULTS = dict(
    CAMERA_SIZE=(1920, 1080),   # (W, H)

    MODEL_PATH="yolov8n.pt",
    YOLO_SIZE=640,
    CONF_THRES=0.30,

    HFOV_DEG=102.0,
    USE_ATAN_MAPPING=True,

    TRACK_TTL_SEC=3.0,
    MATCH_MAX_DEG=25.0,
    PRINT_HZ=8,

    # Plots / UI
    PLOT_WINDOW_SEC=10.0,
    PLOT_UPDATE_HZ=25,

    # Video preview
    SHOW_VIDEO=False,
    VIDEO_W=640,
    VIDEO_UPDATE_HZ=30,
    DRAW_YOLO_BOXES_ON_VIDEO=True,
    DRAW_BEST_OVERLAY_ON_VIDEO=True,

    SHOW_YAW_PLOT=True,
    SHOW_PITCH_PLOT=True,
    SHOW_TEXT_LABELS=True,

    TORCH_THREADS=1,
)


# ---------------- shared schema helpers ----------------
def _ensure_vision_fields(shared: Any) -> None:
    if not hasattr(shared, "vision_lock"):
        shared.vision_lock = threading.Lock()
    if not hasattr(shared, "vision_tracks"):
        shared.vision_tracks = []
    if not hasattr(shared, "vision_dets"):
        shared.vision_dets = []
    if not hasattr(shared, "vision_best"):
        shared.vision_best = None
    if not hasattr(shared, "vision_timestamp"):
        shared.vision_timestamp = 0.0

    # optional shared stop event (some projects use one)
    if not hasattr(shared, "stop_event"):
        shared.stop_event = threading.Event()


def _derive_vfov_deg(hfov_deg: float, img_w: int, img_h: int) -> float:
    return math.degrees(
        2.0 * math.atan(math.tan(math.radians(hfov_deg / 2.0)) * (img_h / img_w))
    )


# ---------------- color helpers ----------------
_label_color = {}

def _qcolor_for_label(label: str):
    if label not in _label_color:
        h = (abs(hash(label)) % 360) / 360.0
        _label_color[label] = pg.hsvColor(h, sat=0.9, val=0.95, alpha=1.0)
    return _label_color[label]

def _bgr_for_label(label: str):
    h = abs(hash(label)) % 180
    hsv = np.array([[[h, 220, 255]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist()
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))


# ---------------- UI Window (non-blocking) ----------------
class VisionWindow(QtWidgets.QMainWindow):
    """
    A normal Qt window that updates using QTimer.
    Does NOT create QApplication and does NOT call exec_().
    """

    def __init__(
        self,
        shared: Any,
        *,
        get_frame_fn,  # callable returning RGB frame or None
        hfov_deg: float,
        vfov_deg: float,
        plot_window_sec: float,
        plot_update_hz: float,

        # initial toggles
        show_yaw_plot: bool,
        show_pitch_plot: bool,
        show_video: bool,
        show_text_labels: bool,
        video_w: int,
        video_update_hz: float,
        draw_yolo_boxes_on_video: bool,
        draw_best_overlay_on_video: bool,
    ):
        super().__init__()
        self.shared = shared
        self.get_frame_fn = get_frame_fn

        self.hfov_deg = float(hfov_deg)
        self.vfov_deg = float(vfov_deg)

        self.plot_window_sec = float(plot_window_sec)
        self.plot_update_hz = float(plot_update_hz)

        # toggles (runtime)
        self.show_yaw_plot = bool(show_yaw_plot)
        self.show_pitch_plot = bool(show_pitch_plot)
        self.show_video = bool(show_video)
        self.show_text_labels = bool(show_text_labels)
        self.draw_yolo_boxes_on_video = bool(draw_yolo_boxes_on_video)
        self.draw_best_overlay_on_video = bool(draw_best_overlay_on_video)

        self.video_w = int(video_w)
        self.video_update_hz = float(video_update_hz)
        self.last_vid = 0.0

        self.setWindowTitle("Vision: Yaw/Pitch Spans + Video (PyQtGraph)")
        self.t0 = time.time()
        self.snapshots = deque()  # (timestamp, tracks_list)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        grid = QtWidgets.QGridLayout(central)

        # ---- Toggle row ----
        ctrl = QtWidgets.QHBoxLayout()

        def add_cb(text, initial, fn):
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(initial)
            cb.stateChanged.connect(lambda _: fn(cb.isChecked()))
            ctrl.addWidget(cb)
            return cb

        add_cb("Yaw plot", self.show_yaw_plot, lambda v: self._set_toggle("show_yaw_plot", v))
        add_cb("Pitch plot", self.show_pitch_plot, lambda v: self._set_toggle("show_pitch_plot", v))
        add_cb("Video", self.show_video, lambda v: self._set_toggle("show_video", v))
        add_cb("Labels", self.show_text_labels, lambda v: self._set_toggle("show_text_labels", v))
        add_cb("YOLO boxes", self.draw_yolo_boxes_on_video, lambda v: self._set_toggle("draw_yolo_boxes_on_video", v))
        add_cb("BEST overlay", self.draw_best_overlay_on_video, lambda v: self._set_toggle("draw_best_overlay_on_video", v))

        ctrl.addStretch(1)
        grid.addLayout(ctrl, 0, 0, 1, 2)

        # ---- Yaw plot ----
        self.plot_yaw = pg.PlotWidget()
        self.plot_yaw.showGrid(x=True, y=True, alpha=0.3)
        self.plot_yaw.setLabel("bottom", "Time", units="s")
        self.plot_yaw.setLabel("left", "Yaw angle", units="deg")
        self.plot_yaw.setYRange(-self.hfov_deg / 2.0 - 5, self.hfov_deg / 2.0 + 5)
        grid.addWidget(self.plot_yaw, 1, 0)

        # ---- Pitch plot ----
        self.plot_pit = pg.PlotWidget()
        self.plot_pit.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pit.setLabel("bottom", "Time", units="s")
        self.plot_pit.setLabel("left", "Pitch angle", units="deg")
        self.plot_pit.setYRange(-self.vfov_deg / 2.0 - 5, self.vfov_deg / 2.0 + 5)
        grid.addWidget(self.plot_pit, 2, 0)

        # ---- Video ----
        self.video_label = QtWidgets.QLabel()
        self.video_label.setMinimumWidth(self.video_w)
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setText("Video preview (toggle ON)")
        grid.addWidget(self.video_label, 1, 1, 2, 1)

        self._apply_visibility()

        # ---- QTimer updates ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(int(1000 / max(1, int(self.plot_update_hz))))

    def _set_toggle(self, name: str, value: bool):
        setattr(self, name, bool(value))
        self._apply_visibility()

    def _apply_visibility(self):
        self.plot_yaw.setVisible(self.show_yaw_plot)
        self.plot_pit.setVisible(self.show_pitch_plot)
        self.video_label.setVisible(self.show_video)

    def update_ui(self):
        if self.show_yaw_plot or self.show_pitch_plot:
            self.update_plots()
        if self.show_video:
            self.update_video()

    def update_plots(self):
        now = time.time()
        with self.shared.vision_lock:
            tr_copy = [dict(t) for t in self.shared.vision_tracks]

        self.snapshots.append((now, tr_copy))
        while self.snapshots and (now - self.snapshots[0][0]) > self.plot_window_sec:
            self.snapshots.popleft()

        if self.show_yaw_plot:
            self.plot_yaw.clear()
        if self.show_pitch_plot:
            self.plot_pit.clear()

        for ts, track_list in self.snapshots:
            x = ts - self.t0
            for tr in track_list:
                c = _qcolor_for_label(tr["label"])
                w = 1 + int(6 * float(tr.get("conf", 0.5)))
                pen = pg.mkPen(c, width=w)

                if self.show_yaw_plot:
                    self.plot_yaw.plot([x, x], [tr["yaw_min"], tr["yaw_max"]], pen=pen)
                if self.show_pitch_plot:
                    self.plot_pit.plot([x, x], [tr["pit_min"], tr["pit_max"]], pen=pen)

        # labels at latest time slice
        if self.show_text_labels and self.snapshots:
            ts_last, tracks_last = self.snapshots[-1]
            x_last = ts_last - self.t0

            for tr in tracks_last:
                c = _qcolor_for_label(tr["label"])
                tag = f'{tr["label"]}#{tr["id"]}'

                if self.show_yaw_plot:
                    ymid = 0.5 * (tr["yaw_min"] + tr["yaw_max"])
                    txt = pg.TextItem(tag, anchor=(0, 0.5))
                    txt.setColor(c)
                    txt.setPos(x_last + 0.05, ymid)
                    self.plot_yaw.addItem(txt)

                if self.show_pitch_plot:
                    ymid = 0.5 * (tr["pit_min"] + tr["pit_max"])
                    txt = pg.TextItem(tag, anchor=(0, 0.5))
                    txt.setColor(c)
                    txt.setPos(x_last + 0.05, ymid)
                    self.plot_pit.addItem(txt)

            if self.show_yaw_plot:
                self.plot_yaw.setXRange(max(0, x_last - self.plot_window_sec), x_last + 1.0, padding=0.0)
            if self.show_pitch_plot:
                self.plot_pit.setXRange(max(0, x_last - self.plot_window_sec), x_last + 1.0, padding=0.0)

    def update_video(self):
        now = time.time()
        if now - self.last_vid < (1.0 / max(1e-6, self.video_update_hz)):
            return
        self.last_vid = now

        frame = self.get_frame_fn()
        if frame is None:
            return

        h, w = frame.shape[:2]
        new_w = self.video_w
        new_h = int(new_w * h / w)
        frame_small = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)  # RGB

        # crosshair at image center
        ax = int(new_w / 2)
        ay = int(new_h / 2)
        cv2.drawMarker(frame_small, (ax, ay), (255, 255, 0),
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)

        # draw boxes
        if self.draw_yolo_boxes_on_video:
            with self.shared.vision_lock:
                dets_copy = list(self.shared.vision_dets)

            sx = new_w / w
            sy = new_h / h

            for d in dets_copy:
                x1 = int(d["x1"] * sx)
                y1 = int(d["y1"] * sy)
                x2 = int(d["x2"] * sx)
                y2 = int(d["y2"] * sy)
                label = d["label"]
                conf = d["conf"]

                bgr = _bgr_for_label(label)
                rgb = (bgr[2], bgr[1], bgr[0])

                cv2.rectangle(frame_small, (x1, y1), (x2, y2), rgb, 2)
                txt = f"{label} {conf:.2f}"
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                y_text = max(0, y1 - th - 6)
                cv2.rectangle(frame_small, (x1, y_text), (x1 + tw + 6, y_text + th + 6), rgb, -1)
                cv2.putText(frame_small, txt, (x1 + 3, y_text + th + 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # best overlay
        if self.draw_best_overlay_on_video:
            with self.shared.vision_lock:
                b = None if self.shared.vision_best is None else dict(self.shared.vision_best)

            if b is not None:
                cxp = int(b["cx"] * (new_w / w))
                cyp = int(b["cy"] * (new_h / h))
                cv2.line(frame_small, (ax, ay), (cxp, cyp), (0, 0, 255), 2)
                cv2.putText(
                    frame_small,
                    f"BEST: {b['label']} c={b['conf']:.2f}  dx={b['dx']:+.0f}px dy={b['dy']:+.0f}px",
                    (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        qimg = QtGui.QImage(
            frame_small.data,
            frame_small.shape[1],
            frame_small.shape[0],
            frame_small.strides[0],
            QtGui.QImage.Format_RGB888,
        )
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))


# ---------------- processor ----------------
class VisionProcessor:
    """
    Starts threads immediately.
    Use create_window() to get a Qt window you can show/hide in run_all.py.
    """

    def __init__(
        self,
        shared_data: Any,
        *,
        poll: float = 0.0,
        camera_size: Tuple[int, int] = DEFAULTS["CAMERA_SIZE"],
        model_path: str = DEFAULTS["MODEL_PATH"],
        yolo_size: int = DEFAULTS["YOLO_SIZE"],
        conf_thres: float = DEFAULTS["CONF_THRES"],
        hfov_deg: float = DEFAULTS["HFOV_DEG"],
        use_atan_mapping: bool = DEFAULTS["USE_ATAN_MAPPING"],
        track_ttl_sec: float = DEFAULTS["TRACK_TTL_SEC"],
        match_max_deg: float = DEFAULTS["MATCH_MAX_DEG"],
        print_hz: float = DEFAULTS["PRINT_HZ"],
        torch_threads: int = DEFAULTS["TORCH_THREADS"],
        picam2: Optional[Picamera2] = None,
    ):
        self.shared = shared_data
        _ensure_vision_fields(self.shared)

        self.poll = float(poll)

        self.CAMERA_SIZE = tuple(camera_size)
        self.IMG_W, self.IMG_H = self.CAMERA_SIZE

        self.MODEL_PATH = model_path
        self.YOLO_SIZE = int(yolo_size)
        self.CONF_THRES = float(conf_thres)

        self.HFOV_DEG = float(hfov_deg)
        self.VFOV_DEG = _derive_vfov_deg(self.HFOV_DEG, self.IMG_W, self.IMG_H)
        self.USE_ATAN_MAPPING = bool(use_atan_mapping)

        self.TRACK_TTL_SEC = float(track_ttl_sec)
        self.MATCH_MAX_DEG = float(match_max_deg)
        self.PRINT_HZ = float(print_hz)

        torch.set_num_threads(int(torch_threads))

        self._running = threading.Event()
        self._running.set()

        self._frame_lock = threading.Lock()
        self._latest_frame = None  # RGB full-res

        print("[Vision] Loading YOLO...")
        self.model = YOLO(self.MODEL_PATH)
        print("[Vision] YOLO loaded.")
        print(f"[Vision] HFOV={self.HFOV_DEG:.2f}°, VFOV={self.VFOV_DEG:.2f}° at {self.IMG_W}x{self.IMG_H}")

        self.picam2 = picam2 if picam2 is not None else Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": self.CAMERA_SIZE})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.8)

        self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True, name="VisionCamera")
        self._yolo_thread = threading.Thread(target=self._yolo_loop, daemon=True, name="VisionYOLO")
        self._cam_thread.start()
        self._yolo_thread.start()

    # ---------- angle mapping ----------
    def _pixel_x_to_angle_deg(self, x: float) -> float:
        if self.USE_ATAN_MAPPING:
            fx = (self.IMG_W / 2.0) / math.tan(math.radians(self.HFOV_DEG / 2.0))
            return math.degrees(math.atan2((x - (self.IMG_W / 2.0)), fx))
        x_norm = (x - (self.IMG_W / 2.0)) / (self.IMG_W / 2.0)
        return x_norm * (self.HFOV_DEG / 2.0)

    # Vertical FIXED sign: top -> +angle, bottom -> -angle
    def _pixel_y_to_angle_deg(self, y: float) -> float:
        if self.USE_ATAN_MAPPING:
            fy = (self.IMG_H / 2.0) / math.tan(math.radians(self.VFOV_DEG / 2.0))
            return -math.degrees(math.atan2((y - (self.IMG_H / 2.0)), fy))
        y_norm = (y - (self.IMG_H / 2.0)) / (self.IMG_H / 2.0)
        return -y_norm * (self.VFOV_DEG / 2.0)

    # ---------- lifecycle ----------
    def stop(self, join_timeout: float = 1.0):
        self._running.clear()
        try:
            self.shared.stop_event.set()
        except Exception:
            pass

        self._cam_thread.join(timeout=join_timeout)
        self._yolo_thread.join(timeout=join_timeout)

        try:
            self.picam2.stop()
        except Exception:
            pass

    def is_alive(self) -> bool:
        return self._cam_thread.is_alive() or self._yolo_thread.is_alive()

    def _should_run(self) -> bool:
        if not self._running.is_set():
            return False
        try:
            if self.shared.stop_event.is_set():
                return False
        except Exception:
            pass
        return True

    def get_latest_frame_rgb(self):
        with self._frame_lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    # ---------- UI creation (NO event loop) ----------
    def create_window(
        self,
        *,
        plot_window_sec: float = DEFAULTS["PLOT_WINDOW_SEC"],
        plot_update_hz: float = DEFAULTS["PLOT_UPDATE_HZ"],
        show_yaw_plot: bool = DEFAULTS["SHOW_YAW_PLOT"],
        show_pitch_plot: bool = DEFAULTS["SHOW_PITCH_PLOT"],
        show_video: bool = DEFAULTS["SHOW_VIDEO"],
        show_text_labels: bool = DEFAULTS["SHOW_TEXT_LABELS"],
        video_w: int = DEFAULTS["VIDEO_W"],
        video_update_hz: float = DEFAULTS["VIDEO_UPDATE_HZ"],
        draw_yolo_boxes_on_video: bool = DEFAULTS["DRAW_YOLO_BOXES_ON_VIDEO"],
        draw_best_overlay_on_video: bool = DEFAULTS["DRAW_BEST_OVERLAY_ON_VIDEO"],
    ) -> VisionWindow:
        return VisionWindow(
            self.shared,
            get_frame_fn=self.get_latest_frame_rgb,
            hfov_deg=self.HFOV_DEG,
            vfov_deg=self.VFOV_DEG,
            plot_window_sec=plot_window_sec,
            plot_update_hz=plot_update_hz,
            show_yaw_plot=show_yaw_plot,
            show_pitch_plot=show_pitch_plot,
            show_video=show_video,
            show_text_labels=show_text_labels,
            video_w=video_w,
            video_update_hz=video_update_hz,
            draw_yolo_boxes_on_video=draw_yolo_boxes_on_video,
            draw_best_overlay_on_video=draw_best_overlay_on_video,
        )

    # ---------- threads ----------
    def _camera_loop(self):
        while self._should_run():
            frame = self.picam2.capture_array()
            frame = frame[:, :, :3]  # RGB
            with self._frame_lock:
                self._latest_frame = frame
            if self.poll > 0:
                time.sleep(self.poll)

    def _yolo_loop(self):
        aim_x = self.IMG_W / 2.0
        aim_y = self.IMG_H / 2.0

        tracks: Dict[int, Dict[str, Any]] = {}
        next_id = 1
        last_print = 0.0

        while self._should_run():
            with self._frame_lock:
                frame = None if self._latest_frame is None else self._latest_frame.copy()

            if frame is None:
                time.sleep(0.005)
                continue

            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            results = self.model(frame_bgr, imgsz=self.YOLO_SIZE, verbose=False)

            boxes = results[0].boxes
            now = time.time()

            detections: List[Dict[str, Any]] = []
            dets_for_publish: List[Dict[str, Any]] = []

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    conf = float(box.conf[0])
                    if conf < self.CONF_THRES:
                        continue

                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)

                    dets_for_publish.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "label": label, "conf": conf})

                    cx = 0.5 * (x1 + x2)
                    cy = 0.5 * (y1 + y2)

                    # yaw span
                    yaw_left = self._pixel_x_to_angle_deg(x1)
                    yaw_right = self._pixel_x_to_angle_deg(x2)
                    yaw_min = min(yaw_left, yaw_right)
                    yaw_max = max(yaw_left, yaw_right)
                    yaw_ctr = 0.5 * (yaw_min + yaw_max)

                    # pitch span (sign-fixed)
                    pit_top = self._pixel_y_to_angle_deg(y1)
                    pit_bot = self._pixel_y_to_angle_deg(y2)
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

            # best snapshot + dx/dy
            if detections:
                best = max(detections, key=lambda d: d["conf"])
                dx = best["cx"] - aim_x  # + right
                dy = best["cy"] - aim_y  # + down
                best_snapshot = dict(best)
                best_snapshot["dx"] = dx
                best_snapshot["dy"] = dy
            else:
                best_snapshot = None

            # match detections to tracks (same label, 2D angular distance)
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

                    if err <= self.MATCH_MAX_DEG and (best_err is None or err < best_err):
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
                    tr["yaw_min"] = det["yaw_min"]; tr["yaw_max"] = det["yaw_max"]; tr["yaw_ctr"] = det["yaw_ctr"]
                    tr["pit_min"] = det["pit_min"]; tr["pit_max"] = det["pit_max"]; tr["pit_ctr"] = det["pit_ctr"]
                    matched_track_ids.add(best_tid)

            # prune dead tracks
            dead = [tid for tid, tr in tracks.items() if (now - tr["last_seen"]) > self.TRACK_TTL_SEC]
            for tid in dead:
                del tracks[tid]

            tracks_list = [dict(t) for t in tracks.values()]

            # publish to shared
            with self.shared.vision_lock:
                self.shared.vision_tracks = tracks_list
                self.shared.vision_dets = dets_for_publish
                self.shared.vision_best = best_snapshot
                self.shared.vision_timestamp = now

            # clean terminal printing
            if self.PRINT_HZ > 0 and (now - last_print) >= (1.0 / self.PRINT_HZ):
                last_print = now
                tr_list_sorted = sorted(tracks.values(), key=lambda x: x["id"])

                if not tr_list_sorted:
                    print("Tracks: (none)")
                else:
                    parts = []
                    for t in tr_list_sorted:
                        parts.append(
                            f"{t['label']}#{t['id']} "
                            f"yaw[{t['yaw_min']:+.1f}..{t['yaw_max']:+.1f}]° "
                            f"pit[{t['pit_min']:+.1f}..{t['pit_max']:+.1f}]° "
                            f"c={t['conf']:.2f}"
                        )
                    print("Tracks: " + " | ".join(parts))

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


# ---------------- factory ----------------
def create_and_run(shared_data: Any, poll: float = 0.0, **kwargs) -> VisionProcessor:
    """
    run_all.py calls this:
        vision = vision_processor.create_and_run(shared_data, poll=0.0)
    """
    return VisionProcessor(shared_data, poll=poll, **kwargs)


# ---------------- standalone test ----------------
if __name__ == "__main__":
    # Standalone only: create a QApplication and show the window.
    import sys
    app = QtWidgets.QApplication(sys.argv)

    class _TmpShared:
        pass

    shared = _TmpShared()
    _ensure_vision_fields(shared)

    vp = create_and_run(shared)

    win = vp.create_window(show_video=True, show_yaw_plot=True, show_pitch_plot=True)
    win.resize(1400, 800)
    win.show()

    try:
        sys.exit(app.exec_())
    finally:
        vp.stop()