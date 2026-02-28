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
CAMERA_SIZE = (1920, 1080)

YOLO_SIZE = 640
CONF_THRES = 0.4
MODEL_PATH = "yolov8n.pt"

HFOV_DEG = 102.0
PLOT_WINDOW_SEC = 10.0

TRACK_TTL_SEC = 1.2
MATCH_MAX_DEG = 8.0
PRINT_HZ = 8

SHOW_TEXT_LABELS = True
USE_ATAN_MAPPING = True

# Preview
SHOW_PREVIEW = False
PREVIEW_W = 640            # a bit bigger since we’re drawing boxes
PREVIEW_FPS_LIMIT = 30

# Plot update rate (and minimize-safe)
PLOT_UPDATE_HZ = 20

torch.set_num_threads(1)

latest_frame = None
running = True
frame_lock = threading.Lock()

snap_lock = threading.Lock()
snapshots = deque()  # (timestamp, list_of_tracks)

# ---- NEW: share latest YOLO detections for preview drawing ----
det_lock = threading.Lock()
latest_dets = []  # list of dicts: {"x1","y1","x2","y2","label","conf"}
# --------------------------------------------------------------

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

# -------- helper: map label -> OpenCV BGR color (stable-ish) --------
def bgr_for_label(label: str):
    # deterministic-ish color from hash
    h = abs(hash(label)) % 360
    # convert HSV->BGR quickly
    hsv = (h, 200, 255)
    bgr = cv2.cvtColor(
        (np.array([[hsv]], dtype=np.uint8)),
        cv2.COLOR_HSV2BGR
    )[0, 0].tolist()
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))

# If numpy isn’t already installed, you probably have it; OpenCV typically pulls it.
import numpy as np

print("Loading YOLO...")
model = YOLO(MODEL_PATH)
print("Model loaded.")

def yolo_worker():
    global latest_frame, running, latest_dets

    img_w = CAMERA_SIZE[0]
    img_h = CAMERA_SIZE[1]

    tracks = {}
    next_id = 1
    last_print = 0.0

    while running:
        if latest_frame is None:
            time.sleep(0.01)
            continue

        with frame_lock:
            frame = latest_frame.copy()

        # Convert RGB -> BGR for YOLO
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = model(frame, imgsz=YOLO_SIZE, verbose=False)
        boxes = results[0].boxes

        detections_for_tracking = []
        dets_for_preview = []
        now = time.time()

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRES:
                    continue

                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Save for preview drawing
                dets_for_preview.append({
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

        # Publish latest detections for preview
        with det_lock:
            latest_dets = dets_for_preview

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

# ---- Plot setup (MAIN THREAD) ----
plt.ion()
fig, ax = plt.subplots()
t0 = time.time()
last_plot = 0.0

def _fig_is_minimized(_fig):
    """
    Best-effort: if backend supports a Tk window, detect minimize and skip redraw.
    If not supported, return False (always draw).
    """
    try:
        mgr = _fig.canvas.manager
        win = getattr(mgr, "window", None)
        if win is None:
            return False
        # Tk: "iconic" when minimized
        state = win.state()
        return state == "iconic"
    except Exception:
        return False

def update_plot():
    global last_plot
    now = time.time()
    if now - last_plot < (1.0 / PLOT_UPDATE_HZ):
        return

    # If minimized, don't redraw (prevents it from un-minimizing / stealing focus)
    if _fig_is_minimized(fig):
        return

    last_plot = now

    with snap_lock:
        snap_copy = list(snapshots)

    ax.cla()
    ax.set_title("Tracked Angular Spans — colored by label (legend = current view + conf range)")
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

    fig.canvas.draw_idle()
    plt.pause(0.001)

# -------- Main --------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": CAMERA_SIZE})
picam2.configure(config)
picam2.start()

if SHOW_PREVIEW:
    cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Preview", PREVIEW_W, int(PREVIEW_W * CAMERA_SIZE[1] / CAMERA_SIZE[0]))
    last_preview = 0.0

yolo_thread = threading.Thread(target=yolo_worker, daemon=True)
yolo_thread.start()

try:
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, :3]
        

        with frame_lock:
            latest_frame = frame

        # Preview (with YOLO boxes)
        if SHOW_PREVIEW:
            now = time.time()
            if now - last_preview >= (1.0 / PREVIEW_FPS_LIMIT):
                last_preview = now

                # OpenCV expects BGR
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Resize for preview
                h, w = bgr.shape[:2]
                new_w = PREVIEW_W
                new_h = int(new_w * h / w)
                small = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)

                # Get latest dets and draw them scaled
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

                    # color per label
                    color = bgr_for_label(label)

                    cv2.rectangle(small, (x1, y1), (x2, y2), color, 2)
                    txt = f"{label} {conf:.2f}"
                    # label background
                    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    y_text = max(0, y1 - th - 6)
                    cv2.rectangle(small, (x1, y_text), (x1 + tw + 6, y_text + th + 6), color, -1)
                    cv2.putText(small, txt, (x1 + 3, y_text + th + 3),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

                cv2.imshow("Preview", small)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # q or ESC
                    break

        # Plot update (main thread; minimize-safe)
        update_plot()

except KeyboardInterrupt:
    print("Stopping...")

finally:
    running = False
    yolo_thread.join(timeout=1.0)
    picam2.stop()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    time.sleep(0.2)