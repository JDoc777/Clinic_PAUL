from picamera2 import Picamera2
from ultralytics import YOLO
import threading
import torch
import time
from collections import deque
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# -------- SETTINGS --------
CAMERA_SIZE = (1920, 1080)

YOLO_SIZE = 320
CONF_THRES = 0.45
MODEL_PATH = "yolov8n.pt"

HFOV_DEG = 102.0
PLOT_WINDOW_SEC = 10.0

TRACK_TTL_SEC = 1.2
MATCH_MAX_DEG = 8.0
PRINT_HZ = 8

SHOW_TEXT_LABELS = True
USE_ATAN_MAPPING = True

# ---- NEW: "giant object" filters (ignore edge-to-edge background rectangles) ----
MAX_BOX_W_FRAC = 0.90      # ignore if bbox width > 90% of image width
MAX_BOX_H_FRAC = 0.90      # ignore if bbox height > 90% of image height
MAX_BOX_AREA_FRAC = 0.70 # ignore if bbox area > 60% of image area
DEBUG_PRINT_HUGE_REJECTS = False  # set True to see what gets rejected
# --------------------------

torch.set_num_threads(1)

latest_frame = None
running = True
frame_lock = threading.Lock()

snap_lock = threading.Lock()
snapshots = deque()  # (timestamp, list_of_tracks)

def pixel_x_to_angle_deg_linear(x, img_w, hfov_deg):
    x_norm = (x - (img_w / 2.0)) / (img_w / 2.0)
    return x_norm * (hfov_deg / 2.0)

def pixel_x_to_angle_deg_atan(x, img_w, hfov_deg):
    fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    return math.degrees(math.atan2((x - (img_w / 2.0)), fx))

def pixel_x_to_angle_deg(x, img_w, hfov_deg):
    return pixel_x_to_angle_deg_atan(x, img_w, hfov_deg) if USE_ATAN_MAPPING else pixel_x_to_angle_deg_linear(x, img_w, hfov_deg)

# ----------------------------
# Color mapping (stable per label)
# ----------------------------
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

    global latest_frame, running
    img_w = CAMERA_SIZE[0]
    img_h = CAMERA_SIZE[1]
    img_area = img_w * img_h

    tracks = {}
    next_id = 1
    last_print = 0.0

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with frame_lock:
            frame = latest_frame.copy()

        results = model(frame, imgsz=YOLO_SIZE, verbose=False)
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

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # ---- NEW: reject giant boxes (edge-to-edge background) ----
                box_w = max(0.0, x2 - x1)
                box_h = max(0.0, y2 - y1)
                box_area = box_w * box_h

                w_frac = box_w / img_w if img_w else 0.0
                h_frac = box_h / img_h if img_h else 0.0
                area_frac = box_area / img_area if img_area else 0.0

                if (w_frac > MAX_BOX_W_FRAC) or (h_frac > MAX_BOX_H_FRAC) or (area_frac > MAX_BOX_AREA_FRAC):
                    if DEBUG_PRINT_HUGE_REJECTS:
                        print(f"REJECT HUGE: {label} conf={conf:.2f} w={w_frac:.2f} h={h_frac:.2f} area={area_frac:.2f}")
                    continue
                # -----------------------------------------------

                a_left  = pixel_x_to_angle_deg(x1, img_w, HFOV_DEG)
                a_right = pixel_x_to_angle_deg(x2, img_w, HFOV_DEG)

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

        # ---- match detections to existing tracks (nearest center angle, same label) ----
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

        # ---- snapshot for plotting ----
        track_list = list(tracks.values())
        with snap_lock:
            snapshots.append((now, track_list))
            while snapshots and (now - snapshots[0][0]) > PLOT_WINDOW_SEC:
                snapshots.popleft()

        # ---- compact terminal snapshot ----
        if now - last_print >= (1.0 / PRINT_HZ):
            last_print = now
            if track_list:
                s = " | ".join(
                    [f"{t['label']}#{t['id']} [{t['a_min']:.1f}..{t['a_max']:.1f}]° (c={t['conf']:.2f})"
                     for t in sorted(track_list, key=lambda x: x["id"])]
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
        ax.set_title("Tracked Angular Spans — colored by label (legend = current view + conf range)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.set_ylim(-HFOV_DEG/2.0 - 5, HFOV_DEG/2.0 + 5)
        ax.grid(True)

        xs_latest = None

        # draw bars for full window
        for ts, track_list in snap_copy:
            x = ts - t0
            xs_latest = x
            for tr in track_list:
                lw = 1.0 + 6.0 * tr["conf"]
                c = get_color_for_label(tr["label"])
                ax.plot([x, x], [tr["a_min"], tr["a_max"]], linewidth=lw, color=c, alpha=0.9)

        # latest snapshot: labels + legend data
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
                cmin = min(confs)
                cmax = max(confs)
                legend_label = f"{lab} (conf {cmin:.2f}-{cmax:.2f})"
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
        frame = frame[:, :, ::-1]  # if colors look wrong, comment out
        with frame_lock:
            latest_frame = frame

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    yolo_thread.join(timeout=1.0)
    picam2.stop()
    time.sleep(0.2)
