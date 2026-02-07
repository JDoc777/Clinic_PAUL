import sys
import signal
import time
import cv2
import argparse
import os
import subprocess
import shutil
import glob
from picamera2 import Picamera2

# Simple Ctrl+C handler to ensure clean shutdown
def _install_sigint_handler(cleanup_fn):
    def _handler(sig, frame):
        cleanup_fn()
        sys.exit(0)
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--camera", type=int, help="camera index to use")
    parser.add_argument("--device", help="fallback v4l2 device path to use when Picamera2 sees no cameras (e.g. /dev/video19)")
    args = parser.parse_args()

    # enumerate available cameras
    def _run_cmd(cmd):
        try:
            out = subprocess.run(cmd, shell=True, check=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=5)
            return out.returncode, out.stdout.strip()
        except Exception as e:
            return 1, str(e)

    try:
        cams = Picamera2.global_camera_info()
    except Exception as e:
        print("Error calling Picamera2.global_camera_info():", e)
        cams = []
    if not cams:
        print("No cameras detected by libcamera / Picamera2. Diagnostic info:")
        print("  PICAMERA_NUM env:", os.environ.get("PICAMERA_NUM"))
        print("  Python executable:", sys.executable)
        print("  Picamera2.global_camera_info() returned:", cams)
        # list /dev/video* nodes
        ret, out = _run_cmd("ls -l /dev/video* 2>/dev/null || true")
        print("  /dev/video*:", out or "(none)")
        # try libcamera helper (if installed)
        if shutil.which("libcamera-hello"):
            ret, out = _run_cmd("libcamera-hello --list-cameras")
            print("  libcamera-hello --list-cameras:\n", out)
        else:
            print("  libcamera-hello not found on PATH")
        # try vcgencmd if available (Raspberry Pi specific)
        if shutil.which("vcgencmd"):
            ret, out = _run_cmd("vcgencmd get_camera")
            print("  vcgencmd get_camera:", out)
        else:
            print("  vcgencmd not found on PATH")
        print("  Check camera ribbon, power, and that libcamera/kernel overlays are enabled (raspi-config).")

        # Fallback: try to open a V4L2 device via OpenCV
        device_arg = args.device or os.environ.get("V4L_DEVICE")
        candidates = [device_arg] if device_arg else sorted(glob.glob("/dev/video*"))
        if not candidates:
            print("No /dev/video* found for fallback. Exiting.")
            return
        print("V4L2 candidates:", candidates)

        cap = None
        chosen = None
        for dev in candidates:
            print("Trying device:", dev)
            # attempt 1: open by path with V4L2 backend
            try:
                tmp = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            except Exception:
                tmp = None
            if tmp and tmp.isOpened():
                cap = tmp; chosen = dev; break
            try:
                if tmp:
                    tmp.release()
            except Exception:
                pass
            # attempt 2: if /dev/videoN, try numeric index with V4L2
            if dev.startswith("/dev/video"):
                try:
                    idx = int(dev.replace("/dev/video", ""))
                    try:
                        tmp = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                    except Exception:
                        tmp = None
                    if tmp and tmp.isOpened():
                        cap = tmp; chosen = dev; break
                    try:
                        if tmp:
                            tmp.release()
                    except Exception:
                        pass
                except Exception:
                    pass
            # attempt 3: open by path without forcing backend
            try:
                tmp = cv2.VideoCapture(dev)
            except Exception:
                tmp = None
            if tmp and tmp.isOpened():
                cap = tmp; chosen = dev; break
            try:
                if tmp:
                    tmp.release()
            except Exception:
                pass

        if not cap or not cap.isOpened():
            print("Failed to open any V4L2 candidate with OpenCV VideoCapture.")
            if shutil.which("v4l2-ctl"):
                for dev in candidates:
                    ret, out = _run_cmd(f"v4l2-ctl --device={dev} --all")
                    print(f"  v4l2-ctl --device={dev} --all:\n", out)
            return

        print("Opened V4L2 device:", chosen)

        def _cleanup_v4l():
            try:
                if cap and cap.isOpened():
                    cap.release()
            except Exception:
                pass
            cv2.destroyAllWindows()
        _install_sigint_handler(_cleanup_v4l)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue
                cv2.imshow(f"V4L2 Preview {chosen} - press q to quit", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            _cleanup_v4l()
        return

    print("Detected cameras:")
    for i, cam in enumerate(cams):
        print(f"  [{i}] {cam.get('Name', cam)}")

    # choose camera from CLI arg, env var, or default 0
    selected = args.camera if args.camera is not None else int(os.environ.get("PICAMERA_NUM", 0))
    if selected < 0 or selected >= len(cams):
        print(f"Invalid camera index {selected}. Available range 0..{len(cams)-1}")
        return

    # construct Picamera2 for selected camera
    try:
        picam2 = Picamera2(camera_num=selected)
    except TypeError:
        picam2 = Picamera2(selected)

    # Configure a preview stream (change size as needed)
    config = picam2.create_preview_configuration({"main":{"format":"RGB888","size":(640,480)}})
    picam2.configure(config)
    picam2.start()

    def _cleanup():
        try:
            if picam2 is not None:
                picam2.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()

    _install_sigint_handler(_cleanup)

    try:
        while True:
            frame = picam2.capture_array()
            # Picamera2 gives RGB; OpenCV expects BGR
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("Picamera2 Preview - press q to quit", bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        _cleanup()

if __name__ == "__main__":
    main()