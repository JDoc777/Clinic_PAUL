import time
import threading

class SonarProcessor:
    """Reads sonar data from a testingUART.UARTSharedData instance."""

    def __init__(self, shared_data, poll=0.05):
        self.shared = shared_data
        self.poll = poll
        self._running = threading.Event()
        self._running.set()
        self._last = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            data = self.shared.get_received_data()
            if data and "sonar" in data:
                sonar = data["sonar"]  # dict with keys: F, B, L, R
                if sonar != self._last:
                    #print(f"Sonar Data: F={sonar['F']} B={sonar['B']} L={sonar['L']} R={sonar['R']}")
                    self._last = sonar
            time.sleep(self.poll)

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

def create_and_run(shared_data, poll=0.05):
    return SonarProcessor(shared_data, poll=poll)

if __name__ == "__main__":
    print("Run this via run_all.py so it gets a shared_data instance from testingUART.")