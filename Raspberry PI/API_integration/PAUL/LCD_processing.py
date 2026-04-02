import time
import threading
import Payload
from DHT_processing import DHTProcessor

class LCDProcessor:
    """Handles LCD updates based on shared data."""

    def __init__(self, shared_data, poll=0.05, cycle_interval=2.0):
        self.shared = shared_data
        self.dht_processor = DHTProcessor(shared_data)
        self.poll = poll
        self.cycle_interval = cycle_interval

        self._running = threading.Event()
        self._running.set()

        self._last_cycle_time = time.time()
        self._current_display_index = 0     # 0 = INSIDE view, 1 = OUTSIDE view

        self._last_text = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def get_fan_bits(self):
        """Extract fan bits from telemetry relay_bits."""
        telemetry = self.shared.get_received_data()
        if telemetry is None:
            return 0, 0

        relay = telemetry.get("relay_bits", 0)

        # In your bit layout:
        # b7 = FAN2
        # b6 = FAN1
        fan1 = (relay >> 6) & 1
        fan2 = (relay >> 7) & 1
        motor_enabled = (relay >> 2) & 1
        return fan1, fan2, motor_enabled

    def _loop(self):
        while self._running.is_set():
            self.display_lcd()
            time.sleep(self.poll)
    
    def get_pi_temperature(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                milli = int(f.read().strip())
            return milli / 1000.0
        except:
            return None

    def display_lcd(self):
        """Cycle between INSIDE and OUTSIDE temperature/humidity pairs."""

        #fan1 = 0
        #fan2 = 1
        fan1, fan2, motor_enabled = self.get_fan_bits()

        # Format the two-line screens
        inside_text = (
            f"IN TEMP: {self.dht_processor.temperatureIN}C  "
            f"IN HUM: {self.dht_processor.humidityIN}%\n"
            "                "  # 16 blank spaces for line 2 (or add status)
        )

        outside_text = (
            f"OUT TEMP: {self.dht_processor.temperatureOUT}C "
            f"OUT HUM: {self.dht_processor.humidityOUT}%\n"
            "                "
        )

        fan_status_text = (
            "   FAN STATUS\n"
            f"  FAN1:{fan1} FAN2:{fan2:<2}"
        )

        # --- Motor Enable Screen ---
        motor_text = "ON " if motor_enabled else "OFF"
        motor_status_text = (
            "  MOTOR STATUS\n"
            f"     EN: {motor_text}"
        )

        # --- Pose Screen ---
        try:
            pos = self.shared.get_received_data()
            x = pos["pose"]["x"]
            y = pos["pose"]["y"]
            theta = pos["pose"]["theta"]
        except:
            x = y = theta = 0

        pose_text = (
            f"X:{x:.2f} Y:{y:.2f}".ljust(16) + "\n" +
            f"TH:{theta:.1f}deg".ljust(16)
        )

        # --- Pi Temp Screen ---
        pi_temp = self.get_pi_temperature()
        if pi_temp is None:
            pi_temp = 0
            
        pi_temp_text = (
            "   PI TEMP\n"
            f"      {int(pi_temp)}C"
        )


        screens = [
            inside_text,
            outside_text,
            fan_status_text,
            motor_status_text,
            pose_text,
            pi_temp_text
        ]


        # Cycle between the two screens
        if time.time() - self._last_cycle_time >= self.cycle_interval:
            self._current_display_index = (self._current_display_index + 1) % len(screens)
            self._last_cycle_time = time.time()

        lcd_text = screens[self._current_display_index]

        # Update only if changed
        if lcd_text != self._last_text:
            Payload.set_text(self.shared, lcd_text)
            self._last_text = lcd_text

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)


def create_and_run(shared_data, poll=0.05):
    return LCDProcessor(shared_data, poll=poll)
