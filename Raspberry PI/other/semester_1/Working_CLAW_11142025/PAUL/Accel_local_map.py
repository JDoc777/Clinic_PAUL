import os
import sys
import time
import threading
import math

# ensure the local PAUL directory is on sys.path so we can import testingUART
sys.path.insert(0, os.path.dirname(__file__))

import testingUART

class AccelGyroProcessor:
    """Reads accel and gyro data from a testingUART.UARTSharedData instance."""

    def __init__(self, shared_data, poll=0.5):
        self.shared = shared_data
        self.poll = poll
        self._running = threading.Event()
        self._running.set()
        self._last_accel = None
        self._last_gyro = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self.degrees = 0.0
        self.theta = 0.0
        self.accel_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self._last_time = time.time()
        self._thread.start()

    def _loop(self):
        target_dt = 0.01  # 100 Hz
        self._last_time = time.time()  # <-- reset right before loop starts
        while self._running.is_set():
            start_time = time.time()

            now = time.time()
            self.dt = now - self._last_time
            self._last_time = now
            if self.dt <= 0 or self.dt > 0.1:
                self.dt = 0.01

            data = self.shared.get_received_data()
            if data:
                accel = data.get("accel")
                gyro = data.get("gyro")

                if not hasattr(self, "bias_initialized") or not self.bias_initialized:
                    self.bias_surge = accel['surge']
                    self.bias_sway  = accel['sway']
                    self.bias_initialized = True
                accel['surge'] -= self.bias_surge
                accel['sway']  -= self.bias_sway

                self.Accel_angle_processing(gyro)


            elapsed = time.time() - start_time
            time.sleep(max(0, target_dt - elapsed))


    def Accel_angle_processing(self, gyro):
        # Compute theta += gyro_yaw * dt
        
        if gyro and "yaw" in gyro:
            if abs(gyro["yaw"]) > 1:  # Only process significant changes
                self.yaw_rad = (gyro["yaw"] * (math.pi / 180))
                self.theta += self.yaw_rad * self.dt
                self.theta = self.theta % (2 * math.pi)
        #print(f"Theta (degrees)!!!!!: {math.degrees(self.theta):.4f}, gyro_yaw={gyro['yaw']:.4f} degrees")
        return self.theta

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

def create_and_run(shared_data, poll=0.5):
    return AccelGyroProcessor(shared_data, poll=poll)

def main(poll_interval=0.5):
    # start reader only (no writer) and skip ascii handshake to avoid sending 'request'
    shared_data, running_event, ser = testingUART.start_reader(start_writer=False, do_handshake=False)
    print("AccelLocalMap: started reader (reader-only). Press Ctrl-C to exit.")
    processor = AccelGyroProcessor(shared_data, poll=poll_interval)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        running_event.clear()
        processor.stop()
        time.sleep(0.2)
        try:
            testingUART.close_serial()
        except Exception:
            pass
        print("Stopped.")

def run():
    """Entry point for run_all: runs main() with default settings."""
    main()

if __name__ == "__main__":
    run()

