#!/usr/bin/env python3
"""
YDLIDAR X2 / X2L viewer with persistent LOG-ODDS occupancy grid (no SDK).

- Reads LiDAR packets over serial (AA55)
- Uses CT bit0 as "new scan" marker
- Accumulates a sweep, then updates a log-odds occupancy grid:
    * Cells along each ray => FREE update (log-odds down)
    * End cell at hit => OCCUPIED update (log-odds up)
- Applies per-sweep decay so old evidence fades over time (moving objects update after a short time)
- Displays occupancy grid as an image in PyQtGraph, with optional live point overlay

Install:
  pip install pyqtgraph pyserial numpy PyQt5
(or PyQt6)
"""

import sys
import struct
from collections import deque

import numpy as np
import serial

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui  # <-- QtGui needed for QTransform



# ---------------- SERIAL / LIDAR CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
RMAX_M = 5.0

READ_CHUNK = 4096
PARSE_LIMIT_PER_TICK = 50
GUI_HZ = 100

START_SCAN_CMD = b"\xA5\x60"
STOP_SCAN_CMD  = b"\xA5\x65"

HEADER = b"\xAA\x55"
FIXED_LEN = 8  # CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)

# Viewer overlay of current sweep points (optional)
SHOW_LIVE_POINTS = True
LIVE_POINT_SIZE = 3
DOWNSAMPLE = 1  # 1=no downsample (parsing), 2=every other point, etc.

# ---------------- OCCUPANCY GRID CONFIG ----------------
# Grid resolution (smaller = finer). 0.02m = 2 cm cells.
GRID_RES_M = 0.02

# Log-odds update strengths (tune these!)
L_OCC = 0.85      # occupied evidence added at hit cell
L_FREE = 0.35     # free evidence subtracted along ray

# Clamp log-odds so probabilities don't saturate too hard
L_MIN = -6.0
L_MAX =  6.0

# Decay per sweep (0 = no forgetting, higher = faster forgetting).
# This decays log-odds toward 0 each sweep: L *= (1 - DECAY_PER_SWEEP)
DECAY_PER_SWEEP = 0.05

# Performance caps:
# Updating ALL points with full ray tracing can be heavy on very fine grids.
# Cap number of rays used per sweep (randomly sampled from points).
MAX_RAYS_PER_SWEEP = 1200

# Max range used for ray tracing (keep consistent with display radius)
RAYTRACE_RMAX = RMAX_M
# -------------------------------------------------------


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
    # Per your original: (raw >> 1) / 64.0
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
    Buffered parser: feed bytes; extract packets.

    Packet format (assumed):
        AA55 + CT + LSN + FSA + LSA + CS + samples(2*LSN)
    """
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        if data:
            self.buf += data
            if len(self.buf) > 200000:
                self.buf = self.buf[-200000:]

    def pop_packet(self):
        """
        Returns: (angles_rad: np.ndarray, dist_m: np.ndarray, is_new_scan: bool)
        or None if no complete packet available yet.
        """
        start = self.buf.find(HEADER)
        if start < 0:
            self.buf = self.buf[-1:]
            return None

        if start > 0:
            del self.buf[:start]

        if len(self.buf) < 2 + FIXED_LEN:
            return None

        fixed = self.buf[2:2 + FIXED_LEN]
        CT, LSN, FSA_raw, LSA_raw, CS = struct.unpack("<BBHHH", fixed)

        if LSN == 0:
            del self.buf[:2]
            return None

        total_len = 2 + FIXED_LEN + (2 * LSN)
        if len(self.buf) < total_len:
            return None

        frame = self.buf[:total_len]
        del self.buf[:total_len]

        samp_bytes = frame[2 + FIXED_LEN:]
        samples = np.frombuffer(samp_bytes, dtype="<u2")
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

        raw = samples
        bad_raw = (raw == 0) | (raw == 0xFFFF)
        dist_m = _decode_dist_m(raw)

        valid = (~bad_raw) & (dist_m > 0.10) & (dist_m < RMAX_M)
        if not np.any(valid):
            return None

        angles_rad = np.deg2rad(angles_deg[valid]).astype(np.float32)
        dist_m = dist_m[valid]

        if DOWNSAMPLE > 1 and dist_m.size > 0:
            angles_rad = angles_rad[::DOWNSAMPLE]
            dist_m = dist_m[::DOWNSAMPLE]

        is_new_scan = bool(CT & 0x01)
        return angles_rad, dist_m, is_new_scan


# ---------------- OCCUPANCY GRID HELPERS ----------------

def logodds_to_prob(L: np.ndarray) -> np.ndarray:
    # p = 1 / (1 + exp(-L))
    return 1.0 / (1.0 + np.exp(-L))


def world_to_grid(x_m: float, y_m: float, origin_i: int, origin_j: int, res: float):
    """
    Convert world meters (x,y) to grid indices (i,j).
    i = row (y), j = col (x)
    """
    j = int(np.floor(x_m / res)) + origin_j
    i = int(np.floor(y_m / res)) + origin_i
    return i, j


def bresenham(i0: int, j0: int, i1: int, j1: int):
    """
    Integer grid traversal from (i0,j0) to (i1,j1).
    Returns arrays of (ii, jj) including both endpoints.
    """
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1

    ii = []
    jj = []

    i, j = i0, j0

    # Standard Bresenham for 2D grid (row/col)
    if dj <= di:
        err = di // 2
        while True:
            ii.append(i)
            jj.append(j)
            if i == i1 and j == j1:
                break
            err -= dj
            if err < 0:
                j += sj
                err += di
            i += si
    else:
        err = dj // 2
        while True:
            ii.append(i)
            jj.append(j)
            if i == i1 and j == j1:
                break
            err -= di
            if err < 0:
                i += si
                err += dj
            j += sj

    return np.array(ii, dtype=np.int32), np.array(jj, dtype=np.int32)


# ---------------- MAIN VIEWER ----------------

class LidarViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL – YDLIDAR X2/X2L Log-Odds Occupancy Grid")
        self.resize(950, 950)

        self.ser = open_serial()
        self.parser = X2Parser()

        # Sweep buffers
        self.angles = deque()
        self.dists  = deque()

        # --- Build occupancy grid ---
        self.res = GRID_RES_M
        self.grid_w = int(np.ceil((2.0 * RMAX_M) / self.res))
        self.grid_h = self.grid_w

        # origin (robot) at center cell
        self.origin_j = self.grid_w // 2
        self.origin_i = self.grid_h // 2

        # log-odds grid (float32)
        self.L = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        # --- UI ---
        pg.setConfigOptions(antialias=False)
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)

        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "X (m)")
        self.plot.setLabel("left", "Y (m)")
        self.plot.setXRange(-RMAX_M, RMAX_M)
        self.plot.setYRange(-RMAX_M, RMAX_M)

        # Occupancy image item
        self.img = pg.ImageItem()
        self.plot.addItem(self.img)

        # Scale pixels -> meters using QTransform (Pi-safe)
        tr = QtGui.QTransform()
        tr.scale(self.res, self.res)
        self.img.setTransform(tr)

        # Place image such that bottom-left is (-RMAX, -RMAX)
        self.img.setPos(-RMAX_M, -RMAX_M)

        # Color map / LUT
        lut = pg.colormap.get("cividis").getLookupTable(0.0, 1.0, 256)
        self.img.setLookupTable(lut)

        # Optional live points overlay
        if SHOW_LIVE_POINTS:
            self.points = pg.ScatterPlotItem(
                size=LIVE_POINT_SIZE,
                pen=None,
                brush=pg.mkBrush(255, 255, 255, 120),
            )
            self.plot.addItem(self.points)
        else:
            self.points = None

        # Robot origin marker
        self.origin_dot = pg.ScatterPlotItem(
            size=10,
            pen=pg.mkPen(width=2),
            brush=pg.mkBrush(255, 255, 255, 180),
        )
        self.origin_dot.setData(pos=np.array([[0.0, 0.0]], dtype=np.float32))
        self.plot.addItem(self.origin_dot)

        # Timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / GUI_HZ))

        # Initial render
        self._render_grid()

    def _process_completed_sweep(self, x_pts: np.ndarray, y_pts: np.ndarray):
        """Update log-odds grid from a completed sweep (x,y arrays in meters)."""
        if x_pts.size == 0:
            return

        # Decay toward 0 (forgetting)
        if DECAY_PER_SWEEP > 0.0:
            self.L *= (1.0 - DECAY_PER_SWEEP)

        # Cap rays for performance
        n = x_pts.size
        if n > MAX_RAYS_PER_SWEEP:
            idx = np.random.choice(n, size=MAX_RAYS_PER_SWEEP, replace=False)
            x = x_pts[idx]
            y = y_pts[idx]
        else:
            x = x_pts
            y = y_pts

        # Clip by raytrace range
        rr = np.hypot(x, y)
        keep = rr < RAYTRACE_RMAX
        x = x[keep]
        y = y[keep]
        if x.size == 0:
            return

        i0, j0 = self.origin_i, self.origin_j
        H, W = self.L.shape

        for xi, yi in zip(x, y):
            i1, j1 = world_to_grid(float(xi), float(yi), self.origin_i, self.origin_j, self.res)

            # Bounds check
            if not (0 <= i1 < H and 0 <= j1 < W):
                continue

            ii, jj = bresenham(i0, j0, i1, j1)
            if ii.size < 2:
                continue

            # Free cells = all except last
            free_ii = ii[:-1]
            free_jj = jj[:-1]

            self.L[free_ii, free_jj] -= L_FREE
            self.L[i1, j1] += L_OCC

        np.clip(self.L, L_MIN, L_MAX, out=self.L)

    def _render_grid(self):
        """Render occupancy probability grid as an image."""
        p = logodds_to_prob(self.L)

        # 0..255 grayscale indices; LUT will colorize
        img8 = (p * 255.0).astype(np.uint8)

        # Transpose to align with plot axes (x horizontal, y vertical)
        self.img.setImage(img8.T, autoLevels=False, levels=(0, 255))

    def tick(self):
        if not self.ser or not self.ser.is_open:
            return

        data = self.ser.read(READ_CHUNK)
        self.parser.feed(data)

        got_any = False

        # Parse packets; when a new scan starts, process the previous sweep
        for _ in range(PARSE_LIMIT_PER_TICK):
            pkt = self.parser.pop_packet()
            if pkt is None:
                break

            angles_rad, dist_m, is_new_scan = pkt

            if is_new_scan:
                # Process prior sweep if we have one
                if len(self.angles) > 0:
                    ang = np.array(self.angles, dtype=np.float32)
                    r   = np.array(self.dists, dtype=np.float32)
                    x = r * np.cos(ang)
                    y = r * np.sin(ang)

                    self._process_completed_sweep(x, y)
                    self._render_grid()

                self.angles.clear()
                self.dists.clear()

            self.angles.extend(angles_rad.tolist())
            self.dists.extend(dist_m.tolist())
            got_any = True

        # Optional live overlay from current partial sweep
        if SHOW_LIVE_POINTS and got_any and len(self.angles) > 0 and self.points is not None:
            ang = np.array(self.angles, dtype=np.float32)
            r   = np.array(self.dists, dtype=np.float32)
            x = r * np.cos(ang)
            y = r * np.sin(ang)
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

    exec_fn = getattr(app, "exec", None) or getattr(app, "exec_", None)
    sys.exit(exec_fn())


if __name__ == "__main__":
    main()
