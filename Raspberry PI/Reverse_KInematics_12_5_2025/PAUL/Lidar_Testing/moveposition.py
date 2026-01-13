#!/usr/bin/env python3
"""
YDLIDAR X2 / X2L viewer with persistent LOG-ODDS occupancy grid + SCAN-MATCH LOCALIZATION (no SDK).

Goal:
- Keep the map frame FIXED (walls stay where they are)
- When you move/rotate the LiDAR, estimate (x, y, theta) by matching the current scan to the existing map
- Then apply log-odds updates into the SAME global map

This uses a simple correlative scan matcher each sweep:
  1) Take endpoints from the current sweep in lidar frame (x_l, y_l)
  2) Search around the previous pose (dx, dy, dtheta) and score how well endpoints land on OCCUPIED cells
  3) Pick best pose -> update pose estimate
  4) Raytrace FREE along rays and OCCUPIED at endpoints at that estimated pose
  5) Time-based decay so moving things fade

Install:
  pip install pyqtgraph pyserial numpy PyQt5
(or PyQt6)
"""

import sys
import struct
import time
from collections import deque

import numpy as np
import serial

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui  # QtGui for QTransform


# ---------------- SERIAL / LIDAR CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
RMAX_M = 8.0

READ_CHUNK = 4096
PARSE_LIMIT_PER_TICK = 50
GUI_HZ = 60

START_SCAN_CMD = b"\xA5\x60"
STOP_SCAN_CMD  = b"\xA5\x65"

HEADER = b"\xAA\x55"
FIXED_LEN = 8  # CT(1) + LSN(1) + FSA(2) + LSA(2) + CS(2)

SHOW_LIVE_POINTS = True
LIVE_POINT_SIZE = 3
DOWNSAMPLE = 1

# ---------------- GLOBAL MAP / OCCUPANCY GRID CONFIG ----------------
# Big fixed map so the world frame does not move when the LiDAR moves.
MAP_SIZE_M = 20.0     # 20m x 20m global map
GRID_RES_M = 0.03     # 3cm cells (0.02 is finer but heavier)

# Log-odds update strengths
L_OCC  = 0.85
L_FREE = 0.35
L_MIN  = -6.0
L_MAX  =  6.0

# Time-based decay using half-life (seconds)
HALF_LIFE_S = 10.0    # bigger = holds longer; smaller = adapts faster

# Performance caps
MAX_RAYS_PER_SWEEP = 1200          # ray updates per sweep
MAX_MATCH_POINTS   = 800           # points used for scan matching score
RAYTRACE_RMAX = RMAX_M

# ---------------- SCAN MATCHING SEARCH WINDOW ----------------
# Two-stage search: coarse then fine around best result.
THETA_WIN_DEG_COARSE = 12.0
THETA_STEP_DEG_COARSE = 2.0
TRANS_WIN_M_COARSE = 0.35
TRANS_STEP_M_COARSE = 0.05

THETA_WIN_DEG_FINE = 4.0
THETA_STEP_DEG_FINE = 1.0
TRANS_WIN_M_FINE = 0.12
TRANS_STEP_M_FINE = 0.02

# Ignore scan matching until map has some structure (first few sweeps)
WARMUP_SWEEPS = 6

# Score uses only "confident occupied" cells from the map
OCC_THRESH = 0.62   # cells above this prob contribute strongly

# ---------------------------------------------------------------


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
    return (raw >> 1) / 64.0


def _decode_dist_m(samples_u16: np.ndarray) -> np.ndarray:
    units = (samples_u16 & 0xFFFC).astype(np.float32)
    dist_mm = units * 0.25
    return (dist_mm / 1000.0).astype(np.float32)


class X2Parser:
    """
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
    return 1.0 / (1.0 + np.exp(-L))


def world_to_grid(x_m: float, y_m: float, origin_i: int, origin_j: int, res: float):
    j = int(np.floor(x_m / res)) + origin_j
    i = int(np.floor(y_m / res)) + origin_i
    return i, j


def bresenham(i0: int, j0: int, i1: int, j1: int):
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1

    ii = []
    jj = []

    i, j = i0, j0

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


def transform_points(xy_lidar: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c, -s],
                  [s,  c]], dtype=np.float32)
    return (xy_lidar @ R.T) + np.array([x, y], dtype=np.float32)


def clamp_angle(theta: float) -> float:
    return (theta + np.pi) % (2*np.pi) - np.pi


# ---------------- SCAN MATCHER ----------------

def score_pose(occ_prob: np.ndarray,
               xy_lidar: np.ndarray,
               pose: tuple,
               origin_i: int,
               origin_j: int,
               res: float,
               occ_thresh: float) -> float:
    x, y, th = pose
    xy_w = transform_points(xy_lidar, x, y, th)

    j = np.floor(xy_w[:, 0] / res).astype(np.int32) + origin_j
    i = np.floor(xy_w[:, 1] / res).astype(np.int32) + origin_i

    H, W = occ_prob.shape
    inside = (i >= 0) & (i < H) & (j >= 0) & (j < W)
    if not np.any(inside):
        return -1e9

    p = occ_prob[i[inside], j[inside]]

    good = p > occ_thresh
    if not np.any(good):
        return float(np.sum(p))

    return float(np.sum(p[good]))


def correlative_scan_match(occ_prob: np.ndarray,
                           xy_lidar: np.ndarray,
                           pose0: tuple,
                           origin_i: int,
                           origin_j: int,
                           res: float) -> tuple:
    x0, y0, th0 = pose0

    def run_search(xc, yc, thc, trans_win, trans_step, th_win_deg, th_step_deg):
        th_win = np.deg2rad(th_win_deg)
        th_step = np.deg2rad(th_step_deg)

        ths = np.arange(thc - th_win, thc + th_win + 1e-9, th_step, dtype=np.float32)
        dxs = np.arange(-trans_win, trans_win + 1e-9, trans_step, dtype=np.float32)
        dys = np.arange(-trans_win, trans_win + 1e-9, trans_step, dtype=np.float32)

        best_pose = (xc, yc, thc)
        best_score = -1e18

        for th in ths:
            th = float(clamp_angle(float(th)))
            for dx in dxs:
                x = float(xc + dx)
                for dy in dys:
                    y = float(yc + dy)
                    sc = score_pose(
                        occ_prob, xy_lidar, (x, y, th),
                        origin_i, origin_j, res,
                        OCC_THRESH
                    )
                    if sc > best_score:
                        best_score = sc
                        best_pose = (x, y, th)
        return best_pose, best_score

    best_pose, _ = run_search(
        x0, y0, th0,
        TRANS_WIN_M_COARSE, TRANS_STEP_M_COARSE,
        THETA_WIN_DEG_COARSE, THETA_STEP_DEG_COARSE
    )

    best_pose, _ = run_search(
        best_pose[0], best_pose[1], best_pose[2],
        TRANS_WIN_M_FINE, TRANS_STEP_M_FINE,
        THETA_WIN_DEG_FINE, THETA_STEP_DEG_FINE
    )

    return best_pose


# ---------------- MAIN VIEWER ----------------

class LidarViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL – YDLIDAR X2/X2L Log-Odds + Scan-Match Localization")
        self.resize(980, 980)

        self.ser = open_serial()
        self.parser = X2Parser()

        # Sweep buffers
        self.angles = deque()
        self.dists  = deque()

        # --- Global map grid ---
        self.res = GRID_RES_M
        self.map_size = MAP_SIZE_M
        self.grid_w = int(np.ceil(self.map_size / self.res))
        self.grid_h = self.grid_w

        self.origin_j = self.grid_w // 2
        self.origin_i = self.grid_h // 2

        self.L = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        # Pose estimate (x,y,theta) in world
        self.pose = (0.0, 0.0, 0.0)
        self.sweep_count = 0

        # Time-based decay
        self.last_decay_t = time.time()

        # --- UI ---
        pg.setConfigOptions(antialias=False)
        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)

        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "X (m)")
        self.plot.setLabel("left", "Y (m)")

        half = self.map_size / 2.0
        self.plot.setXRange(-half, half)
        self.plot.setYRange(-half, half)

        self.img = pg.ImageItem()
        self.plot.addItem(self.img)

        tr = QtGui.QTransform()
        tr.scale(self.res, self.res)
        self.img.setTransform(tr)

        self.img.setPos(-half, -half)

        lut = pg.colormap.get("viridis").getLookupTable(0.0, 1.0, 256)
        self.img.setLookupTable(lut)

        if SHOW_LIVE_POINTS:
            self.points = pg.ScatterPlotItem(
                size=LIVE_POINT_SIZE,
                pen=None,
                brush=pg.mkBrush(255, 255, 255, 110),
            )
            self.plot.addItem(self.points)
        else:
            self.points = None

        self.robot_dot = pg.ScatterPlotItem(
            size=9,
            pen=pg.mkPen(width=2),
            brush=pg.mkBrush(255, 255, 255, 180),
        )
        self.plot.addItem(self.robot_dot)

        self.robot_heading = self.plot.plot([], [], pen=pg.mkPen(width=2))

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(int(1000 / GUI_HZ))

        self._render_grid()
        self._render_robot()

    def _apply_time_decay(self):
        now = time.time()
        dt = float(now - self.last_decay_t)
        self.last_decay_t = now

        if dt <= 0 or HALF_LIFE_S <= 0:
            return

        decay = 0.5 ** (dt / float(HALF_LIFE_S))
        self.L *= np.float32(decay)

    def _process_completed_sweep_with_pose(self, xy_lidar: np.ndarray):
        if xy_lidar.size == 0:
            return

        self._apply_time_decay()

        occ_prob = logodds_to_prob(self.L)

        n = xy_lidar.shape[0]
        if n > MAX_MATCH_POINTS:
            idx = np.random.choice(n, size=MAX_MATCH_POINTS, replace=False)
            xy_match = xy_lidar[idx]
        else:
            xy_match = xy_lidar

        self.sweep_count += 1
        if self.sweep_count > WARMUP_SWEEPS:
            self.pose = correlative_scan_match(
                occ_prob=occ_prob,
                xy_lidar=xy_match,
                pose0=self.pose,
                origin_i=self.origin_i,
                origin_j=self.origin_j,
                res=self.res
            )

        x_r, y_r, th_r = self.pose

        # Ray update point cap
        if n > MAX_RAYS_PER_SWEEP:
            idx = np.random.choice(n, size=MAX_RAYS_PER_SWEEP, replace=False)
            xy_use = xy_lidar[idx]
        else:
            xy_use = xy_lidar

        rr = np.hypot(xy_use[:, 0], xy_use[:, 1])
        xy_use = xy_use[rr < RAYTRACE_RMAX]
        if xy_use.size == 0:
            return

        xy_w = transform_points(xy_use, x_r, y_r, th_r)

        i0, j0 = world_to_grid(x_r, y_r, self.origin_i, self.origin_j, self.res)
        H, W = self.L.shape
        if not (0 <= i0 < H and 0 <= j0 < W):
            return

        for xe, ye in xy_w:
            i1, j1 = world_to_grid(float(xe), float(ye), self.origin_i, self.origin_j, self.res)
            if not (0 <= i1 < H and 0 <= j1 < W):
                continue

            ii, jj = bresenham(i0, j0, i1, j1)
            if ii.size < 2:
                continue

            self.L[ii[:-1], jj[:-1]] -= np.float32(L_FREE)
            self.L[i1, j1] += np.float32(L_OCC)

        np.clip(self.L, L_MIN, L_MAX, out=self.L)

    def _render_grid(self):
        p = logodds_to_prob(self.L)
        img8 = (p * 255.0).astype(np.uint8)
        self.img.setImage(img8.T, autoLevels=False, levels=(0, 255))

    def _render_robot(self):
        x, y, th = self.pose
        self.robot_dot.setData(pos=np.array([[x, y]], dtype=np.float32))
        hx = x + 0.4 * np.cos(th)
        hy = y + 0.4 * np.sin(th)
        self.robot_heading.setData([x, hx], [y, hy])

    def tick(self):
        if not self.ser or not self.ser.is_open:
            return

        data = self.ser.read(READ_CHUNK)
        self.parser.feed(data)

        for _ in range(PARSE_LIMIT_PER_TICK):
            pkt = self.parser.pop_packet()
            if pkt is None:
                break

            angles_rad, dist_m, is_new_scan = pkt

            if is_new_scan:
                if len(self.angles) > 0:
                    ang = np.array(self.angles, dtype=np.float32)
                    r   = np.array(self.dists, dtype=np.float32)
                    x_l = r * np.cos(ang)
                    y_l = r * np.sin(ang)
                    xy_l = np.column_stack((x_l, y_l)).astype(np.float32)

                    self._process_completed_sweep_with_pose(xy_l)
                    self._render_grid()
                    self._render_robot()

                    if SHOW_LIVE_POINTS and self.points is not None and xy_l.shape[0] > 0:
                        x_r, y_r, th_r = self.pose
                        xy_w = transform_points(xy_l, x_r, y_r, th_r)
                        self.points.setData(pos=xy_w)

                self.angles.clear()
                self.dists.clear()

            self.angles.extend(angles_rad.tolist())
            self.dists.extend(dist_m.tolist())

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
