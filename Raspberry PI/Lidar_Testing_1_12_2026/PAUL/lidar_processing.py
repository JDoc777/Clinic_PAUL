#!/usr/bin/env python3
"""
lidar_processing.py  (PAUL-style, NO testingUART changes)

- LiDAR is directly connected to the Pi (USB serial).
- We DO NOT use shared_data["..."] (item assignment) because UARTSharedData doesn't support it.
- We keep the SAME "processor" pattern as your other modules:
    * LidarProcessor starts its own thread
    * create_and_run() helper
    * debug prints (rate-limited)
    * get_latest_scan() for other modules (run_all / lidar_local_map) to read

Optional publish (safe):
- If shared_data exists, we try to publish via ATTRIBUTE assignment:
      shared_data.lidar_scan = payload
  (This does NOT require changing testingUART.py)
"""

import time
import struct
import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import serial

# ---------------- SERIAL / LIDAR CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

START_SCAN_CMD = b"\xA5\x60"
STOP_SCAN_CMD  = b"\xA5\x65"

HEADER = b"\xAA\x55"
FIXED_LEN = 8  # CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)

READ_CHUNK = 8192

# Valid range filtering
RMIN_M = 0.10
RMAX_M = 5.0

# Loop tuning
POLL_S = 0.001
PARSE_LIMIT = 200

# Debug printing
DEBUG_PRINT_PERIOD_S = 0.5

# Shared publish key (attribute on shared_data)
SHARED_ATTR_NAME = "lidar_scan"


def open_serial(port: str = SERIAL_PORT, baudrate: int = BAUDRATE) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.01,
    )
    ser.reset_input_buffer()
    try:
        ser.write(START_SCAN_CMD)
        ser.flush()
    except Exception:
        pass
    return ser


def _angle_deg_from_raw(raw: int) -> float:
    # Per your odds_graph: (raw >> 1) / 64.0
    return (raw >> 1) / 64.0


def _decode_dist_m(samples_u16: np.ndarray) -> np.ndarray:
    """
    Distance samples: lower 2 bits often flags; units in 0.25 mm.
    """
    units = (samples_u16 & 0xFFFC).astype(np.float32)
    dist_mm = units * 0.25
    return (dist_mm / 1000.0).astype(np.float32)


class X2Parser:
    """
    Buffered packet parser for YDLIDAR X2/X2L.

    pop_packet() -> (angles_rad, ranges_m, is_new_scan) or None
    """
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        if data:
            self.buf += data
            if len(self.buf) > 200000:
                self.buf = self.buf[-200000:]

    def pop_packet(self) -> Optional[Tuple[np.ndarray, np.ndarray, bool]]:
        start = self.buf.find(HEADER)
        if start < 0:
            self.buf = self.buf[-1:]
            return None
        if start > 0:
            del self.buf[:start]

        if len(self.buf) < 2 + FIXED_LEN:
            return None

        fixed = self.buf[2:2 + FIXED_LEN]
        CT, LSN, FSA_raw, LSA_raw, _CS = struct.unpack("<BBHHH", fixed)
        if LSN == 0:
            del self.buf[:2]
            return None

        total_len = 2 + FIXED_LEN + (2 * LSN)
        if len(self.buf) < total_len:
            return None

        frame = self.buf[:total_len]
        del self.buf[:total_len]

        samples = np.frombuffer(frame[2 + FIXED_LEN:], dtype="<u2")
        if samples.size != LSN:
            return None

        start_deg = _angle_deg_from_raw(FSA_raw)
        end_deg   = _angle_deg_from_raw(LSA_raw)
        if end_deg < start_deg:
            end_deg += 360.0

        if LSN > 1:
            angles_deg = (np.linspace(start_deg, end_deg, LSN, dtype=np.float32) % 360.0)
        else:
            angles_deg = np.array([start_deg % 360.0], dtype=np.float32)

        bad_raw = (samples == 0) | (samples == 0xFFFF)
        ranges_m = _decode_dist_m(samples)

        valid = (~bad_raw) & (ranges_m > RMIN_M) & (ranges_m < RMAX_M)
        if not np.any(valid):
            return None

        angles_rad = np.deg2rad(angles_deg[valid]).astype(np.float32)
        ranges_m = ranges_m[valid].astype(np.float32)

        is_new_scan = bool(CT & 0x01)
        return angles_rad, ranges_m, is_new_scan


@dataclass
class LidarScan:
    angles_rad: np.ndarray
    ranges_m: np.ndarray
    timestamp: float


class LidarProcessor:
    """
    PAUL-style threaded LiDAR processor.

    - Stores latest COMPLETE sweep internally (thread-safe).
    - run_all.py can print by calling get_latest_scan().
    - lidar_local_map.py can consume the same way.
    - Optional: publishes to shared_data as an ATTRIBUTE (not dict assignment).
    """
    def __init__(self, shared_data=None, poll: float = POLL_S, debug: bool = False, publish_shared: bool = True):
        self.shared = shared_data
        self.poll = float(poll)
        self.debug = bool(debug)
        self.publish_shared = bool(publish_shared)

        self.parser = X2Parser()
        self.ser: Optional[serial.Serial] = None

        self._angles_buf = deque()
        self._ranges_buf = deque()

        self._lock = threading.Lock()
        self._latest: Optional[LidarScan] = None

        self.sweep_count = 0
        self.last_error: Optional[str] = None

        self._running = threading.Event()
        self._running.set()

        self._last_print_t = 0.0

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 1.0):
        self._running.clear()
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(STOP_SCAN_CMD)
                    self.ser.flush()
                except Exception:
                    pass
                self.ser.close()
        except Exception:
            pass
        self._thread.join(timeout=join_timeout)

    def get_latest_scan(self) -> Optional[LidarScan]:
        with self._lock:
            if self._latest is None:
                return None
            return LidarScan(
                angles_rad=self._latest.angles_rad.copy(),
                ranges_m=self._latest.ranges_m.copy(),
                timestamp=self._latest.timestamp,
            )

    def _publish_to_shared_attr(self, payload: dict):
        """
        Safe shared publish:
        - DOES NOT use shared_data["key"] = ...
        - Tries attribute assignment: shared_data.lidar_scan = payload
        - If shared_data has a lock, we use it.
        """
        if (self.shared is None) or (not self.publish_shared):
            return
        try:
            lock = getattr(self.shared, "lock", None)
            if lock is not None:
                with lock:
                    setattr(self.shared, SHARED_ATTR_NAME, payload)
            else:
                setattr(self.shared, SHARED_ATTR_NAME, payload)
        except Exception:
            # Don't spam warnings; shared publish is optional
            pass

    def _finalize_sweep(self):
        if len(self._angles_buf) == 0:
            self._angles_buf.clear()
            self._ranges_buf.clear()
            return

        ang = np.array(self._angles_buf, dtype=np.float32)
        rng = np.array(self._ranges_buf, dtype=np.float32)

        self._angles_buf.clear()
        self._ranges_buf.clear()

        scan = LidarScan(angles_rad=ang, ranges_m=rng, timestamp=time.time())
        with self._lock:
            self._latest = scan

        self.sweep_count += 1

        # Optional: publish to shared_data as attribute
        self._publish_to_shared_attr({
            "angles_rad": scan.angles_rad,
            "ranges_m": scan.ranges_m,
            "timestamp": scan.timestamp,
            "sweep_count": self.sweep_count,
        })

        # Debug print (rate-limited)
        if self.debug:
            now = time.time()
            if (now - self._last_print_t) >= DEBUG_PRINT_PERIOD_S:
                self._last_print_t = now
                if rng.size > 0:
                    close = int(np.sum(rng < 0.50))
                    print(
                        f"[LiDAR] sweeps={self.sweep_count} pts={rng.size} "
                        f"r=[{float(rng.min()):.2f},{float(rng.max()):.2f}] close<0.50m={close}",
                        flush=True
                    )

    def _loop(self):
        try:
            self.ser = open_serial(SERIAL_PORT, BAUDRATE)
            if self.debug:
                print(f"[LiDAR] opened {SERIAL_PORT} @ {BAUDRATE}", flush=True)
        except Exception as e:
            self.last_error = f"open_serial failed: {e}"
            print(f"[LiDAR][ERR] {self.last_error}", flush=True)
            return

        while self._running.is_set():
            try:
                data = self.ser.read(READ_CHUNK)
                self.parser.feed(data)

                for _ in range(PARSE_LIMIT):
                    pkt = self.parser.pop_packet()
                    if pkt is None:
                        break

                    angles_rad, ranges_m, is_new_scan = pkt

                    if is_new_scan:
                        self._finalize_sweep()

                    self._angles_buf.extend(angles_rad.tolist())
                    self._ranges_buf.extend(ranges_m.tolist())

            except Exception as e:
                self.last_error = str(e)
                if self.debug:
                    print(f"[LiDAR][WARN] {e}", flush=True)
                time.sleep(0.1)

            time.sleep(self.poll)


def create_and_run(shared_data=None, poll=POLL_S, debug=False, publish_shared=True, **kwargs) -> LidarProcessor:
    return LidarProcessor(shared_data=shared_data, poll=poll, debug=debug, publish_shared=publish_shared)


if __name__ == "__main__":
    print("Run via run_all.py (LiDAR is USB -> Pi, not Arduino UART).")
