import time
import cv2
from picamera2 import Picamera2
from ultralytics import YOLO


def main():
    # -------- SETTINGS --------
    CAMERA_SIZE = (640, 480)
    YOLO_SIZE = 320
    CONF_THRES = 0.35
    DETECT_EVERY = 3
    PERSIST_FRAMES = 6
    MODEL_PATH = "yolov8n.pt"
    # --------------------------

    print("Loading YOLO model...")
    model = YOLO(MODEL_PATH)
    print("Model loaded.")

    # -------- Camera Setup --------
    picam = Picamera2()

    config = picam.create_video_configuration(
        main={
            "size": CAMERA_SIZE,
            "format": "RGB888"   # FORCE 3-channel RGB
        }
    )

    picam.configure(config)
    picam.start()
    print("Camera started.")

    last_dets = []
    persist_left = 0
    frame_idx = 0

    # FPS tracking
    fps_t0 = time.time()
    fps_frames = 0
    fps_val = 0.0

    try:
        while True:
            # Capture frame (RGB)
            frame = picam.capture_array()

            # Convert RGB → BGR for OpenCV + YOLO
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame_idx += 1
            annotated = frame.copy()

            # Run detection every N frames
            if frame_idx % DETECT_EVERY == 0:
                resized = cv2.resize(frame, (YOLO_SIZE, YOLO_SIZE))

                results = model(resized, verbose=False)
                boxes = results[0].boxes

                dets = []

                if boxes is not None:
                    h, w = frame.shape[:2]

                    for box in boxes:
                        conf = float(box.conf[0])
                        if conf < CONF_THRES:
                            continue

                        cls_id = int(box.cls[0])
                        label = model.names[cls_id]

                        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]

                        # Scale back to original frame
                        x1 = int(x1 / YOLO_SIZE * w)
                        x2 = int(x2 / YOLO_SIZE * w)
                        y1 = int(y1 / YOLO_SIZE * h)
                        y2 = int(y2 / YOLO_SIZE * h)

                        dets.append((x1, y1, x2, y2, label, conf))

                if len(dets) > 0:
                    last_dets = dets
                    persist_left = PERSIST_FRAMES
                else:
                    persist_left = max(0, persist_left - 1)

            # Draw detections (persist between inference frames)
            if persist_left > 0:
                for (x1, y1, x2, y2, label, conf) in last_dets:
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        annotated,
                        f"{label} {conf:.2f}",
                        (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                persist_left -= 1

            # FPS calculation
            fps_frames += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps_val = fps_frames / (now - fps_t0)
                fps_t0 = now
                fps_frames = 0

            cv2.putText(
                annotated,
                f"FPS: {fps_val:.1f}",
                (10, annotated.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.imshow("PiCam YOLO", annotated)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        picam.stop()
        print("Camera stopped.")


if __name__ == "__main__":
    main()
