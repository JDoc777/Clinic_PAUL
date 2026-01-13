import sys
import time
import cv2
import os

def main():
	# Try Picamera2 first (recommended on modern Raspberry Pi OS)
	try:
		from picamera2 import Picamera2
		picam2_available = True
	except Exception:
		picam2_available = False

	window_name = "Camera Preview - press q to quit, s to save"

	if picam2_available:
		# Picamera2 path
		from picamera2 import Picamera2
		from picamera2.encoders import JpegEncoder
		from picamera2.outputs import FileOutput

		picam2 = Picamera2()
		# small preview size to keep CPU usage reasonable; change as needed
		config = picam2.create_preview_configuration({"main": {"size": (1280, 720)}})
		picam2.configure(config)
		picam2.start()
		time.sleep(0.2)  # allow sensor to settle

		cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
		img_count = 0
		try:
			while True:
				frame = picam2.capture_array()
				# frame from Picamera2 is RGB; convert to BGR for OpenCV display
				bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
				cv2.imshow(window_name, bgr)
				key = cv2.waitKey(1) & 0xFF
				if key == ord('q') or key == 27:
					break
				if key == ord('s'):
					filename = f"capture_{int(time.time())}_{img_count}.jpg"
					cv2.imwrite(filename, bgr)
					print("Saved", filename)
					img_count += 1
		finally:
			picam2.stop()
			cv2.destroyAllWindows()
	else:
		# Fallback to OpenCV VideoCapture (V4L2). Device index may need to change.
		cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
		if not cap.isOpened():
			# try without explicit backend
			cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			print("Failed to open camera. If you're using Picamera2, install it (pip) and enable the camera. Exiting.")
			sys.exit(1)

		# Optionally set resolution; comment/uncomment as needed
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

		cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
		img_count = 0
		try:
			while True:
				ret, frame = cap.read()
				if not ret:
					print("Failed to grab frame")
					break
				cv2.imshow(window_name, frame)
				key = cv2.waitKey(1) & 0xFF
				if key == ord('q') or key == 27:
					break
				if key == ord('s'):
					filename = f"capture_{int(time.time())}_{img_count}.jpg"
					cv2.imwrite(filename, frame)
					print("Saved", filename)
					img_count += 1
		finally:
			cap.release()
			cv2.destroyAllWindows()

if __name__ == "__main__":
	main()