import time
import threading
import Payload

class DHTProcessor:
    """Reads DHT data (DHTO and DHTI) from a testingUART.UARTSharedData instance."""

    def __init__(self, shared_data, poll=0.05):
        self.shared = shared_data
        self.poll = poll
        self._running = threading.Event()
        self._running.set()
        self._last = {"DHTO": None, "DHTI": None}
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.temperatureOUT = None
        self.humidityOUT = None
        self.temperatureIN = None
        self.humidityIN = None

    def _loop(self):
        while self._running.is_set():
            data = self.shared.get_received_data()
            if data:
                dhto = data.get("DHTO")
                dhti = data.get("DHTI")

                if dhto != self._last["DHTO"] or dhti != self._last["DHTI"]:
                    self._last["DHTO"] = dhto
                    self._last["DHTI"] = dhti

            if self._last["DHTO"] is not None and self._last["DHTI"] is not None:
                self.FanControl(self._last["DHTO"], self._last["DHTI"])

            time.sleep(self.poll)

    def _format_dht(self, dht):
        """Format DHT data for printing."""
        if isinstance(dht, dict):
            try:
                temp = float(dht.get("temp", "0"))
                hum = float(dht.get("hum", "0"))
                return f"temp={temp:.1f}°C, hum={hum:.1f}%"
            except ValueError:
                return "Invalid data"
        return "No data"

    def FanControl(self, dhto, dhti):
        """Control a fan based on temperature and humidity."""
        try:
            self.temperatureOUT = float(dhto.get("temp")) if isinstance(dhto, dict) else None
            self.humidityOUT = float(dhto.get("hum")) if isinstance(dhto, dict) else None
            self.temperatureIN = float(dhti.get("temp")) if isinstance(dhti, dict) else None
            self.humidityIN = float(dhti.get("hum")) if isinstance(dhti, dict) else None
        except (ValueError, TypeError):
            #print("Invalid DHT data: Unable to convert to float")
            return

        if self.temperatureOUT is not None and self.humidityOUT is not None and self.temperatureIN is not None and self.humidityIN is not None:
            if ((self.temperatureOUT > 35 or (self.temperatureIN >= self.temperatureOUT + 2)) and self.humidityOUT < 85):
                Payload.set_flags_byte(self.shared, 0b01101111)  
            else:
                #print(self.temperatureIN, self.temperatureOUT)
                #print("Turning fan OFF")
                Payload.set_flags_byte(self.shared, 0b00101111)
        else:
            pass
            #print(self.temperatureIN, self.temperatureOUT)
            #print("Invalid DHT data: temperature or humidity is None")

    def stop(self, join_timeout=1.0):
        """Stop the DHT processor."""
        self._running.clear()
        self._thread.join(timeout=join_timeout)

def create_and_run(shared_data, poll=0.05):
    """Factory function to create and start a DHTProcessor."""
    return DHTProcessor(shared_data, poll=poll)

if __name__ == "__main__":
    print("Run this via run_all.py so it gets a shared_data instance from testingUART.")