from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

while True:
    frame = picam2.capture_array()

    results = model(frame)

    annotated = results[0].plot()

    cv2.imshow("YOLO Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
