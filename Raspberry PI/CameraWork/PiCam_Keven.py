import time
import cv2
from picamera2 import Picamera2
import YOLO as YOLO


# ----------------------------
# YOLO detection + drawing
# ----------------------------
def yolo_detect_and_draw(
    frame_bgr,
    model,
    yolo_size=320,
    conf_thres=0.35
):
    """
    Runs YOLO on a resized copy of frame and draws detections on the original frame.
    frame_bgr: OpenCV BGR image from picam.capture_array()
    """
    h, w = frame_bgr.shape[:2]
    annotated = frame_bgr.copy()

    # YOLO expects images; Ultralytics handles BGR fine, but we resize for speed
    resized = cv2.resize(frame_bgr, (yolo_size, yolo_size), interpolation=cv2.INTER_LINEAR)

    # Run inference
    results = model(resized, verbose=False)
    boxes = results[0].boxes

    if boxes is None or len(boxes) == 0:
        # Optional: show message when no detections
        cv2.putText(
            annotated,
            "No detections",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2
        )
        return annotated, []  # no detections

    dets = []
    for box in boxes:
        conf = float(box.conf[0])
        if conf < conf_thres:
            continue

        cls_id = int(box.cls[0])
        label = model.names[cls_id]

        # coords are in resized space -> scale back to original frame
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        x1 = int(x1 / yolo_size * w)
        x2 = int(x2 / yolo_size * w)
        y1 = int(y1 / yolo_size * h)
        y2 = int(y2 / yolo_size * h)

        dets.append((x1, y1, x2, y2, label, conf))

    # Draw
    for (x1, y1, x2, y2, label, conf) in dets:
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{label} {conf:.2f}"
        cv2.putText(
            annotated,
            text,
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    return annotated, dets


def draw_cached_dets(frame_bgr, dets):
    """Draw previously computed detections onto the current frame."""
    annotated = frame_bgr.copy()
    for (x1, y1, x2, y2, label, conf) in dets:
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{label} {conf:.2f}"
        cv2.putText(
            annotated,
            text,
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )
    return annotated


# ----------------------------
# Main
# ----------------------------
def main():
    # --------- TUNABLE SETTINGS ----------
    CAMERA_SIZE = (640, 480)   # smaller = smoother
    YOLO_SIZE = 320            # 256/320/416 (smaller = faster)
    CONF_THRES = 0.35          # lower if missing detections
    DETECT_EVERY = 3           # run YOLO every N frames (2..6 typical)
    PERSIST_FRAMES = 6         # keep last boxes for this many frames
    MODEL_PATH = "yolov8n.pt"  # tiny/nano is best on Pi
    # -------------------------------------

    # Load YOLO model (nano is fastest)
    model = YOLO(MODEL_PATH)

    # Setup PiCamera2
    picam = Picamera2()
    config = picam.create_video_configuration(main={"size": CAMERA_SIZE})
    picam.configure(config)
    picam.start()

    last_dets = []
    persist_left = 0
    frame_idx = 0

    # FPS counter
    fps_t0 = time.time()
    fps_frames = 0
    fps_val = 0.0

    try:
        while True:
            frame = picam.capture_array()  # BGR

            frame_idx += 1

            # Run YOLO only every N frames
            if frame_idx % DETECT_EVERY == 0:
                annotated, dets = yolo_detect_and_draw(
                    frame,
                    model,
                    yolo_size=YOLO_SIZE,
                    conf_thres=CONF_THRES
                )

                # Cache detections so boxes persist between inference frames
                if len(dets) > 0:
                    last_dets = dets
                    persist_left = PERSIST_FRAMES
                else:
                    # no dets this inference frame; decay persistence
                    persist_left = max(0, persist_left - 1)

            else:
                # Re-draw last boxes to keep them visible
                if persist_left > 0 and len(last_dets) > 0:
                    annotated = draw_cached_dets(frame, last_dets)
                    persist_left -= 1
                else:
                    annotated = frame

            # FPS overlay
            fps_frames += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps_val = fps_frames / (now - fps_t0)
                fps_t0 = now
                fps_frames = 0

            cv2.putText(
                annotated,
                f"FPS: {fps_val:.1f}  detect_every={DETECT_EVERY}  yolo_size={YOLO_SIZE}",
                (10, annotated.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.imshow("PiCam YOLO Smooth", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        picam.stop()


if __name__ == "__main__":
    main()