from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()

# Simple preview configuration
config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)
picam2.configure(config)

# Start preview window
picam2.start_preview(Preview.QTGL)
picam2.start()

print("Camera preview running. Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

picam2.stop()
picam2.stop_preview()
