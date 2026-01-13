#!/usr/bin/env python3
"""
YDLIDAR X2 / X2L live viewer using PyQtGraph (no SDK).

- Opens LiDAR over serial (CP210x/ttyUSB0)
- Sends generic start-scan command (0xA5 0x60) if accepted
- Parses X2/X2L packets (AA55 header)
- Uses CT bit0 to detect start of a new 360° sweep
- Displays ONLY the latest sweep
- Connects nearby points into line segments ("walls")

Install (once):
    pip install pyqtgraph pyserial numpy PyQt5
or (if you have Qt6):
    pip install pyqtgraph pyserial numpy PyQt6
"""

import sys
import struct
from collections import deque

import numpy as np
import serial

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore  # works for PyQt5/PyQt6 through pyqtgraph


# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200          # X2/X2L typical default
RMAX_M = 8.0               # display radius (meters)
MAX_POINTS = 5000          # max points stored per sweep
GUI_HZ = 60                # GUI refresh rate
READ_CHUNK = 4096          # bytes to read from serial each tick
PARSE_LIMIT_PER_TICK = 50  # max packets to parse per tick (prevents UI stalls)
WALL_GAP_M = 0.10          # break line if neighbor gap > this (meters)
DOWNSAMPLE = 1             # 1 = no downsample, 2 = every other point, etc.
START_SCAN_CMD = b"\xA5\x60"
STOP_SCAN_CMD  = b"\xA5\x65"

HEADER = b"\xAA\x55"
FIXED_LEN = 8  # CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)
# ----------------------------------------


def open_serial() -> serial.Serial:
    """Open serial port and attempt start-scan command."""
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,
        )
        ser.reset_input_buffer()
        print(f"[INFO] Opened {SERIAL_PORT} @ {BAUDRATE}")

        # Start scan (some models accept this, some ignore; safe either way)
        try:
            ser.write(START_SCAN_CMD)
            ser.flush()
            print("[INFO] Sent start-scan command (A5 60)")
        except Exception as e:
            print(f"[WARN] Start-scan command failed/ignored: {e}")

        return ser
    except Exception as e:
        raise RuntimeError(f"Failed to open {SERIAL_PORT}: {e}") from e


def _angle_deg_from_raw(raw: int) -> float:
    # Per your comment: (raw >> 1) / 64.0
    return (raw >> 1) / 64.0


def _decode_dist_m(samples_u16: np.ndarray) -> np.ndarray:
    """
    Distance samples: lower 2 bits often flags; units in 0.25 mm.
    """
    units = (samples_u16 & 0xFFFC).astype(np.float32)
    dist_mm = units * 0.25
    return dist_mm / 1000.0


class X2Parser:
    """
    Buffered parser: feed bytes; extract packets.

    Packet format (assumed):
        AA55 + CT + LSN + FSA + LSA + CS + samples(2*LSN)
    """
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        if data:
            self.buf += data
            # keep buffer bounded (prevents memory creep if sync lost)
            if len(self.buf) > 200000:
                self.buf = self.buf[-200000:]

    def pop_packet(self):
        """
        Returns: (angles_rad: np.ndarray, dist_m: np.ndarray, is_new_scan: bool)
        or None if no complete packet available yet.
        """
        # Find header
        start = self.buf.find(HEADER)
        if start < 0:
            # no header: keep last byte in case it is 0xAA
            self.buf = self.buf[-1:]
            return None

        # Drop leading junk
        if start > 0:
            del self.buf[:start]

        # Need header + fixed fields
        if len(self.buf) < 2 + FIXED_LEN:
            return None

        # Unpack fixed fields after AA55
        fixed = self.buf[2:2 + FIXED_LEN]
        CT, LSN, FSA_raw, LSA_raw, CS = struct.unpack("<BBHHH", fixed)

        if LSN == 0:
            # discard header and continue
            del self.buf[:2]
            return None

        total_len = 2 + FIXED_LEN + (2 * LSN)
        if len(self.buf) < total_len:
            return None

        frame = self.buf[:total_len]
        del self.buf[:total_len]

        # Samples
        samp_bytes = frame[2 + FIXED_LEN:]
        samples = np.frombuffer(samp_bytes, dtype="<u2")  # little-endian uint16
        if samples.size != LSN:
            return None

        # Angles (FIX: handle 360° wrap)
        start_deg = _angle_deg_from_raw(FSA_raw)
        end_deg   = _angle_deg_from_raw(LSA_raw)

        # Example wrap: 355° -> 3° should become 355° -> 363°
        if end_deg < start_deg:
            end_deg += 360.0

        if LSN > 1:
            angles_deg = (np.linspace(start_deg, end_deg, LSN, dtype=np.float32) % 360.0)
        else:
            angles_deg = np.array([start_deg % 360.0], dtype=np.float32)

        # Distances
        raw = samples  # uint16 array

        # FIX: common invalid / no-return markers
        bad_raw = (raw == 0) | (raw == 0xFFFF)

        dist_m = _decode_dist_m(raw)

        # Filter: reject bad + too-near + too-far
        valid = (~bad_raw) & (dist_m > 0.10) & (dist_m < RMAX_M)
        if not np.any(valid):
            return None

        angles_rad = np.deg2rad(angles_deg[valid])
        dist_m = dist_m[valid]

        # optional downsample
        if DOWNSAMPLE > 1 and dist_m.size > 0:
            angles_rad = angles_rad[::DOWNSAMPLE]
            dist_m = dist_m[::DOWNSAMPLE]

        is_new_scan = bool(CT & 0x01)
        return angles_rad, dist_m, is_new_scan


class LidarViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL – YDLIDAR X2/X2L Live Viewer")
        self.resize(900, 900)

        self.ser = open_serial()
        self.parser = X2Parser()

        # Store only latest sweep
        self.angles = deque(maxlen=MAX_POINTS)
        self.dists  = deque(maxlen=MAX_POINTS)

        # ---- UI ----
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)

        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "X (m)")
        self.plot.setLabel("left", "Y (m)")
        self.plot.setXRange(-RMAX_M, RMAX_M)
        self.plot.setYRange(-RMAX_M, RMAX_M)

        # Line for walls + faint points on top
        self.wall_line = self.plot.plot([], [], pen=pg.mkPen(width=2))
        self.points = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(200, 200, 200, 130))
        self.plot.addItem(self.points)

        # ---- Timer ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / GUI_HZ))

    def tick(self):
        if not self.ser or not self.ser.is_open:
            return

        # Read a chunk and feed parser
        data = self.ser.read(READ_CHUNK)
        self.parser.feed(data)

        got_any = False

        # Parse multiple packets per tick, but cap work
        for _ in range(PARSE_LIMIT_PER_TICK):
            pkt = self.parser.pop_packet()
            if pkt is None:
                break
            angles_rad, dist_m, is_new_scan = pkt

            if is_new_scan:
                self.angles.clear()
                self.dists.clear()

            self.angles.extend(angles_rad.tolist())
            self.dists.extend(dist_m.tolist())
            got_any = True

        if not got_any or not self.angles:
            return

        ang = np.array(self.angles, dtype=np.float32)
        r   = np.array(self.dists, dtype=np.float32)

        # Polar -> Cartesian
        x = r * np.cos(ang)
        y = r * np.sin(ang)

        # Build wall line segments by angle-ordering and inserting NaNs on gaps
        order = np.argsort(ang)
        xs = x[order]
        ys = y[order]

        gaps = np.hypot(np.diff(xs), np.diff(ys))
        xs_line = xs.copy()
        ys_line = ys.copy()

        break_idxs = np.where(gaps > WALL_GAP_M)[0]
        if break_idxs.size:
            xs_line[break_idxs + 1] = np.nan
            ys_line[break_idxs + 1] = np.nan

        self.wall_line.setData(xs_line, ys_line)
        self.points.setData(pos=np.column_stack((x, y)))

    def closeEvent(self, event):
        print("[INFO] Closing viewer...")
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(STOP_SCAN_CMD)
                    self.ser.flush()
                except Exception:
                    pass
                self.ser.close()
        finally:
            event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    viewer = LidarViewer()
    viewer.show()

    # Qt5 uses exec_(), Qt6 uses exec(). pyqtgraph's Qt wrapper maps correctly,
    # but this is the safest across environments.
    exec_fn = getattr(app, "exec", None) or getattr(app, "exec_", None)
    sys.exit(exec_fn())


if __name__ == "__main__":
    main()
