from picamera2 import Picamera2
from ultralytics import YOLO
import threading
import torch
import time
from collections import deque
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import cv2

# -------- SETTINGS --------
MAIN_SIZE  = (1920, 1080)     # keep this for "full FOV / angle math"
LORES_SIZE = (1720, 1080)       # run YOLO + preview on this (fast)

YOLO_IMGSZ = 384           # 192/256 recommended for Pi
MODEL_PATH = "yolov8n.pt"

# Let more boxes survive NMS, but not crazy low (0.05 was heavy/noisy)
PREDICT_CONF = 0.20
PREDICT_IOU  = 0.75
AGNOSTIC_NMS = False

# Final filter after boxes survive NMS
CONF_THRES = 0.25

HFOV_DEG = 102.0
PLOT_WINDOW_SEC = 10.0

TRACK_TTL_SEC = 1.2
MATCH_MAX_DEG = 8.0
PRINT_HZ = 8

MIN_HIT_STREAK = 3

SHOW_TEXT_LABELS = True
USE_ATAN_MAPPING = True

MAX_BOX_W_FRAC = 0.95
MAX_BOX_H_FRAC = 0.95
MAX_BOX_AREA_FRAC = 0.80
DEBUG_PRINT_HUGE_REJECTS = False

# Preview (cheap now because it's lores)
ENABLE_PREVIEW = True
PREVIEW_EVERY_N_FRAMES = 5    # can be small since lores is cheap
DRAW_TRACK_SPANS_ON_PREVIEW = True
# --------------------------

torch.set_num_threads(1)

latest_lores = None
running = True
frame_lock = threading.Lock()

track_lock = threading.Lock()
latest_confirmed_tracks = []

snap_lock = threading.Lock()
snapshots = deque()  # (timestamp, list_of_confirmed_tracks)

def pixel_x_to_angle_deg_linear(x, img_w, hfov_deg):
    x_norm = (x - (img_w / 2.0)) / (img_w / 2.0)
    return x_norm * (hfov_deg / 2.0)

def pixel_x_to_angle_deg_atan(x, img_w, hfov_deg):
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    return math.degrees(math.atan2((x - (img_w / 2.0)), fx))

def pixel_x_to_angle_deg(x, img_w, hfov_deg):
    return pixel_x_to_angle_deg_atan(x, img_w, hfov_deg) if USE_ATAN_MAPPING else pixel_x_to_angle_deg_linear(x, img_w, hfov_deg)

def angle_deg_to_pixel_x(angle_deg, img_w, hfov_deg):
    if USE_ATAN_MAPPING:
        fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        return (img_w / 2.0) + fx * math.tan(math.radians(angle_deg))
    else:
        return (img_w / 2.0) * (1.0 + (angle_deg / (hfov_deg / 2.0)))

# Color mapping per label
label_color = {}
_color_list = plt.rcParams['axes.prop_cycle'].by_key().get('color', ['blue'])
_color_index = 0

def get_color_for_label(label: str) -> str:
    global _color_index
    if label not in label_color:
        label_color[label] = _color_list[_color_index % len(_color_list)]
        _color_index += 1
    return label_color[label]

print("Loading YOLO...")
model = YOLO(MODEL_PATH)
print("Model loaded.")

def yolo_worker():
    global latest_lores, running, latest_confirmed_tracks
    main_w, main_h = MAIN_SIZE
    lo_w, lo_h = LORES_SIZE
    sx = main_w / lo_w
    sy = main_h / lo_h
    img_area_main = main_w * main_h

    tracks = {}
    next_id = 1
    last_print = 0.0

    while running:
        if latest_lores is None:
            time.sleep(0.01)
            continue

        with frame_lock:
            frame_lo = latest_lores.copy()

        # YOLO on lores frame
        results = model.predict(
            frame_lo,
            imgsz=YOLO_IMGSZ,
            conf=PREDICT_CONF,
            iou=PREDICT_IOU,
            agnostic_nms=AGNOSTIC_NMS,
            verbose=False
        )
        boxes = results[0].boxes

        detections = []
        now = time.time()

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                # lores coords
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # scale to MAIN coords for angle math / size filters
                x1m = x1 * sx
                x2m = x2 * sx
                y1m = y1 * sy
                y2m = y2 * sy

                box_w = max(0.0, x2m - x1m)
                box_h = max(0.0, y2m - y1m)
                box_area = box_w * box_h

                w_frac = box_w / main_w
                h_frac = box_h / main_h
                area_frac = box_area / img_area_main

                if (w_frac > MAX_BOX_W_FRAC) or (h_frac > MAX_BOX_H_FRAC) or (area_frac > MAX_BOX_AREA_FRAC):
                    if DEBUG_PRINT_HUGE_REJECTS:
                        print(f"REJECT HUGE: {label} conf={conf:.2f} w={w_frac:.2f} h={h_frac:.2f} area={area_frac:.2f}")
                    continue

                a_left  = pixel_x_to_angle_deg(x1m, main_w, HFOV_DEG)
                a_right = pixel_x_to_angle_deg(x2m, main_w, HFOV_DEG)

                a_min = min(a_left, a_right)
                a_max = max(a_left, a_right)
                a_ctr = 0.5 * (a_min + a_max)

                detections.append({
                    "label": label,
                    "conf": conf,
                    "a_min": a_min,
                    "a_max": a_max,
                    "a_ctr": a_ctr
                })

        # ---- match to tracks ----
        matched_track_ids = set()

        for det in detections:
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
                    "last_seen": now,
                    "hits": 1,
                    "confirmed": False,
                }
                matched_track_ids.add(tid)
            else:
                tr = tracks[best_tid]
                tr["a_min"] = det["a_min"]
                tr["a_max"] = det["a_max"]
                tr["a_ctr"] = det["a_ctr"]
                tr["conf"] = det["conf"]
                tr["last_seen"] = now
                tr["hits"] += 1
                if tr["hits"] >= MIN_HIT_STREAK:
                    tr["confirmed"] = True
                matched_track_ids.add(best_tid)

        # penalize unmatched
        for tid, tr in tracks.items():
            if tid not in matched_track_ids:
                tr["hits"] = max(0, tr["hits"] - 1)
                if tr["hits"] == 0:
                    tr["confirmed"] = False

        # prune old
        dead = [tid for tid, tr in tracks.items() if (now - tr["last_seen"]) > TRACK_TTL_SEC]
        for tid in dead:
            del tracks[tid]

        confirmed = [t for t in tracks.values() if t["confirmed"]]

        with track_lock:
            latest_confirmed_tracks = [
                {"id": t["id"], "label": t["label"], "conf": t["conf"], "a_min": t["a_min"], "a_max": t["a_max"]}
                for t in confirmed
            ]

        with snap_lock:
            snapshots.append((now, confirmed))
            while snapshots and (now - snapshots[0][0]) > PLOT_WINDOW_SEC:
                snapshots.popleft()

        if now - last_print >= (1.0 / PRINT_HZ):
            last_print = now
            if confirmed:
                s = " | ".join(
                    [f"{t['label']}#{t['id']} [{t['a_min']:.1f}..{t['a_max']:.1f}]° (c={t['conf']:.2f})"
                     for t in sorted(confirmed, key=lambda x: x["id"])]
                )
                print(s)

        time.sleep(0.02)

def plot_worker():
    plt.ion()
    fig, ax = plt.subplots()
    t0 = time.time()

    while running:
        with snap_lock:
            snap_copy = list(snapshots)

        ax.cla()
        ax.set_title("Tracked Angular Spans — colored by label")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.set_ylim(-HFOV_DEG/2.0 - 5, HFOV_DEG/2.0 + 5)
        ax.grid(True)

        xs_latest = None

        for ts, track_list in snap_copy:
            x = ts - t0
            xs_latest = x
            for tr in track_list:
                lw = 1.0 + 6.0 * tr["conf"]
                c = get_color_for_label(tr["label"])
                ax.plot([x, x], [tr["a_min"], tr["a_max"]], linewidth=lw, color=c, alpha=0.9)

        active_handles = []
        if snap_copy:
            ts_last, tracks_last = snap_copy[-1]
            x_last = ts_last - t0

            conf_by_label = {}
            for tr in tracks_last:
                conf_by_label.setdefault(tr["label"], []).append(tr["conf"])

            if SHOW_TEXT_LABELS:
                for tr in tracks_last:
                    c = get_color_for_label(tr["label"])
                    y_mid = 0.5 * (tr["a_min"] + tr["a_max"])
                    ax.text(x_last + 0.05, y_mid, f"{tr['label']}#{tr['id']}", color=c, fontsize=8, va="center")

            for lab, confs in sorted(conf_by_label.items()):
                c = get_color_for_label(lab)
                legend_label = f"{lab} (conf {min(confs):.2f}-{max(confs):.2f})"
                active_handles.append(Line2D([0], [0], color=c, lw=3, label=legend_label))

        if active_handles:
            ax.legend(handles=active_handles, loc="upper left", fontsize=8)

        if xs_latest is not None:
            ax.set_xlim(max(0, xs_latest - PLOT_WINDOW_SEC), xs_latest + 1.0)
        else:
            ax.set_xlim(0, PLOT_WINDOW_SEC)

        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

# -------- Main --------
picam2 = Picamera2()

# Use main+lores streams (huge performance win)
config = picam2.create_preview_configuration(
    main={"size": MAIN_SIZE, "format": "RGB888"},
    lores={"size": LORES_SIZE, "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

yolo_thread = threading.Thread(target=yolo_worker, daemon=True)
yolo_thread.start()

plot_thread = threading.Thread(target=plot_worker, daemon=True)
plot_thread.start()

frame_count = 0

try:
    while True:
        # Grab lores only (fast) for YOLO + preview
        lo = picam2.capture_array("lores")

        with frame_lock:
            latest_lores = lo

        if ENABLE_PREVIEW:
            frame_count += 1
            if frame_count % PREVIEW_EVERY_N_FRAMES == 0:
                disp = lo.copy()

                if DRAW_TRACK_SPANS_ON_PREVIEW:
                    with track_lock:
                        tr_copy = list(latest_confirmed_tracks)

                    lo_w, lo_h = LORES_SIZE
                    main_w, _ = MAIN_SIZE
                    sx_back = lo_w / main_w  # convert MAIN-pixel x to LORES pixel x

                    for tr in tr_copy:
                        # convert angles -> MAIN pixel x -> LORES pixel x
                        px_left_main  = angle_deg_to_pixel_x(tr["a_min"], main_w, HFOV_DEG)
                        px_right_main = angle_deg_to_pixel_x(tr["a_max"], main_w, HFOV_DEG)
                        px_left  = int(round(px_left_main * sx_back))
                        px_right = int(round(px_right_main * sx_back))

                        px_left = max(0, min(lo_w - 1, px_left))
                        px_right = max(0, min(lo_w - 1, px_right))

                        cv2.line(disp, (px_left, 0), (px_left, lo_h - 1), (0, 255, 255), 2)
                        cv2.line(disp, (px_right, 0), (px_right, lo_h - 1), (0, 255, 255), 2)

                        txt = f"{tr['label']}#{tr['id']} {tr['conf']:.2f}"
                        cv2.putText(disp, txt, (min(px_left, px_right) + 5, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Picamera2 RGB888 -> OpenCV expects BGR
                disp_bgr = cv2.cvtColor(disp, cv2.COLOR_RGB2BGR)
                cv2.imshow("Preview (lores)", disp_bgr)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    yolo_thread.join(timeout=1.0)
    picam2.stop()
    time.sleep(0.2)
    if ENABLE_PREVIEW:
        cv2.destroyAllWindows()