import sys
import math
import time
import serial
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QCheckBox, QFrame
)
from PyQt5.QtGui import QMatrix4x4
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl

# ============================================================
# CHASSIS / TURRET
# ============================================================
CHASSIS = {"Lx": 356.0, "Ly": 356.0, "Lz": 203.0}
TURRET  = {"radius": 70.0, "height": 35.0}

BASE_HEIGHT             = 76.0
BASE_TO_SHOULDER_OFFSET = 0.0

# Arm link lengths (mm)
L1 = 203.0 #160
L2 = 228.6
L3 = 120.0

# Claw
CLAW_HINGE_SEP     = 16.0
FINGER_LEN         = 5.0
TIP_ARC_RADIUS     = 5.0
TIP_ARC_PTS        = 10
PINCH_DISTANCE_MM  = 4.0
GRIP_SMOOTH        = 0.12
CLAW_CENTER_OFFSET = 11.0

HOME_XYZ = np.array([
    160,
    0,
    CHASSIS["Lz"] + 120
], dtype=float)

# Obstacle half-extents (mm)
OBS_HALF = np.array([25.0, 25.0, 35.0], dtype=float)

# ============================================================
# JOINT LIMITS
# ============================================================
JOINT_LIMITS = {
    "yaw":      (math.radians(-90),  math.radians(90)),
    "shoulder": (math.radians(-45),  math.radians(180)),
    "elbow":    (math.radians(-180), math.radians(0)),
    "wrist":    (math.radians(-90),  math.radians(90)),
}

HOME_ANGLES = [
    0.0,
    math.radians(180),
    math.radians(-170),
    math.radians(-10)
]

SIM_JOINT_SPEEDS = {
    "yaw":      0.05,
    "shoulder": 0.07,
    "elbow":    0.08,
    "wrist":    0.06,
}

WARN_YELLOW = 0.80
WARN_RED    = 0.98

BOX_MARGIN = 3.0
CYL_MARGIN = 8.0

PATH_CHECK_STEPS = 10

VIA_YAW_N      = 7
VIA_SHOULDER_N = 7
VIA_ELBOW_N    = 7

# ============================================================
# MG996R PWM constants
# ============================================================
MG996R_PW_MIN   = 500
MG996R_PW_MAX   = 2500
MG996R_PERIOD   = 20000
MG996R_DEG_MIN  = 0.0
MG996R_DEG_MAX  = 180.0

# ============================================================
# SERVO CALIBRATION
# Map simulation joint angle (deg) to real servo command (deg)
#
# servo_cmd = center_deg + direction * sim_joint_deg + offset_deg
#
# center_deg:
#   Real servo angle when the SIM joint is at 0 degrees.
#
# direction:
#   +1  -> servo increases when sim angle increases
#   -1  -> servo decreases when sim angle increases
#
# offset_deg:
#   Extra trim value you can tweak after testing.
# ============================================================
SERVO_CAL = {
    "yaw": {
        "center_deg": 120,
        "direction": 1,
        "scale": 1.0,
        "offset_deg": 0,
        "min_deg": 0,
        "max_deg": 180,
    },
    "shoulder": {
        "center_deg": 90,
        "direction": -1,
        "scale": 0.9,
        "offset_deg": 80,
        "min_deg": 0,
        "max_deg": 180,
    },
    "elbow": {
        "center_deg": 90,
        "direction": -1,
        "scale": 0.7,
        "offset_deg": -55,
        "min_deg": 0,
        "max_deg": 180,
    },
    "wrist": {
        "center_deg": 90,
        "direction": 1,
        "scale": 0.75,
        "offset_deg": 0,
        "min_deg": 0,
        "max_deg": 180,
    },
    "claw": {
        "open_deg": 0,
        "closed_deg": 180,
        "offset_deg": 0,
        "min_deg": 0,
        "max_deg": 180,
    }
}

# ============================================================
# SERIAL SETTINGS
# ============================================================
SERIAL_PORT = "COM11"
SERIAL_BAUD = 115200


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_to_pwm(joint_name, angle_rad):
    lo, hi = JOINT_LIMITS[joint_name]
    span = hi - lo
    if abs(span) < 1e-9:
        norm = 0.5
    else:
        norm = clamp((angle_rad - lo) / span, 0.0, 1.0)
    pw_us = MG996R_PW_MIN + norm * (MG996R_PW_MAX - MG996R_PW_MIN)
    duty  = (pw_us / MG996R_PERIOD) * 100.0
    return pw_us, duty


def angle_to_pwm255(joint_name, angle_rad):
    lo, hi = JOINT_LIMITS[joint_name]
    mid    = (lo + hi) / 2.0
    half   = (hi - lo) / 2.0
    if half < 1e-9:
        return 0
    return clamp(round((angle_rad - mid) / half * 255), -255, 255)


def sim_angle_to_servo_deg(joint_name, angle_rad):
    cfg = SERVO_CAL[joint_name]
    sim_deg = math.degrees(angle_rad)

    servo_deg = (
        cfg["center_deg"] +
        cfg["direction"] * cfg.get("scale", 1.0) * sim_deg +
        cfg.get("offset_deg", 0)
    )

    servo_deg = clamp(servo_deg, cfg["min_deg"], cfg["max_deg"])
    return int(round(servo_deg))


def claw_grip_to_servo_deg(grip_value):
    """
    Convert claw grip state [0..1] to calibrated claw servo angle.
        0.0 = closed
        1.0 = open
    """
    cfg = SERVO_CAL["claw"]
    grip_value = clamp(grip_value, 0.0, 1.0)

    servo_deg = (
        cfg["closed_deg"] +
        grip_value * (cfg["open_deg"] - cfg["closed_deg"]) +
        cfg.get("offset_deg", 0)
    )
    servo_deg = clamp(servo_deg, cfg["min_deg"], cfg["max_deg"])
    return int(round(servo_deg))


def current_angles_to_servos(angles, claw_deg=None, grip_value=None):
    names = ["yaw", "shoulder", "elbow", "wrist"]
    servo_vals = [sim_angle_to_servo_deg(n, a) for n, a in zip(names, angles)]

    if claw_deg is None:
        if grip_value is None:
            claw_deg = claw_grip_to_servo_deg(1.0)
        else:
            claw_deg = claw_grip_to_servo_deg(grip_value)

    servo_vals.append(int(round(clamp(claw_deg, 0, 180))))
    return tuple(servo_vals)


def unit(v):
    n = float(np.linalg.norm(v))
    return v / (n + 1e-9)


LIMIT_BUFFER = math.radians(5)


def within_limits(yaw, shoulder, elbow, wrist):
    buf = LIMIT_BUFFER
    return (
        JOINT_LIMITS["yaw"][0]      + buf <= yaw      <= JOINT_LIMITS["yaw"][1]      - buf and
        JOINT_LIMITS["shoulder"][0] + buf <= shoulder <= JOINT_LIMITS["shoulder"][1] - buf and
        JOINT_LIMITS["elbow"][0]    + buf <= elbow    <= JOINT_LIMITS["elbow"][1]    - buf and
        JOINT_LIMITS["wrist"][0]    + buf <= wrist    <= JOINT_LIMITS["wrist"][1]    - buf
    )


JOINT_HOMES = {
    "yaw":      0.0,
    "shoulder": math.radians(180),
    "elbow":    math.radians(-170),
    "wrist":    0.0,
}


def joint_usage_fraction(name, value):
    lo, hi = JOINT_LIMITS[name]
    home   = JOINT_HOMES[name]
    half   = min(abs(home - lo), abs(hi - home))
    return abs(value - home) / half if half > 1e-9 else 0.0


def limit_color_style(fraction):
    if fraction >= WARN_RED:
        return "color: #ff3030; font-weight: bold;"
    elif fraction >= WARN_YELLOW:
        return "color: #ffcc00; font-weight: bold;"
    return "color: #80ff80;"


def get_arm_mount(cp):
    return np.array([cp[0], cp[1], CHASSIS["Lz"]], dtype=float)


def get_deadzone_box(cp):
    return {
        "center": np.array([cp[0], cp[1], CHASSIS["Lz"] / 2.0], dtype=float),
        "size":   np.array([
            CHASSIS["Lx"] + 2 * BOX_MARGIN,
            CHASSIS["Ly"] + 2 * BOX_MARGIN,
            CHASSIS["Lz"] + 2 * BOX_MARGIN
        ], dtype=float)
    }


def get_deadzone_cyl(cp):
    return {
        "center": np.array([cp[0], cp[1], CHASSIS["Lz"]], dtype=float),
        "radius": TURRET["radius"] + CYL_MARGIN,
        "height": TURRET["height"] + 2 * CYL_MARGIN
    }


def make_box_mesh(center, size_xyz, color=(0.3, 0.3, 0.35, 0.6)):
    sx, sy, sz = size_xyz
    cx, cy, cz = center
    x0, x1 = cx - sx/2, cx + sx/2
    y0, y1 = cy - sy/2, cy + sy/2
    z0, z1 = cz - sz/2, cz + sz/2
    verts = np.array([
        [x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
        [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1],
    ], dtype=float)
    faces = np.array([
        [0,1,2],[0,2,3],[4,6,5],[4,7,6],
        [0,4,5],[0,5,1],[1,5,6],[1,6,2],
        [2,6,7],[2,7,3],[3,7,4],[3,4,0],
    ], dtype=int)
    md = gl.MeshData(vertexes=verts, faces=faces)
    item = gl.GLMeshItem(
        meshdata=md, smooth=False, drawFaces=True, drawEdges=True,
        edgeColor=(1.0, 1.0, 1.0, 1.0)
    )
    item.setColor(color)
    return item


def make_cylinder_mesh(center, radius, height, slices=36, color=(0.45, 0.45, 0.5, 0.7)):
    cx, cy, cz = center
    z0, z1 = cz, cz + height
    angles = np.linspace(0, 2*np.pi, slices, endpoint=False)
    c0 = np.stack([cx + radius*np.cos(angles), cy + radius*np.sin(angles), np.full_like(angles, z0)], axis=1)
    c1 = np.stack([cx + radius*np.cos(angles), cy + radius*np.sin(angles), np.full_like(angles, z1)], axis=1)
    verts = np.vstack([c0, c1, [[cx, cy, z0], [cx, cy, z1]]])
    ic0, ic1 = 2*slices, 2*slices+1
    faces = []
    for i in range(slices):
        j = (i + 1) % slices
        faces += [[i, j, slices + j], [i, slices + j, slices + i]]
    for i in range(slices):
        j = (i + 1) % slices
        faces.append([ic0, j, i])
    for i in range(slices):
        j = (i + 1) % slices
        faces.append([ic1, slices + i, slices + j])
    md = gl.MeshData(vertexes=verts, faces=np.array(faces, dtype=int))
    item = gl.GLMeshItem(meshdata=md, smooth=True, drawFaces=True, drawEdges=False)
    item.setColor(color)
    return item


def seg_vs_box(p0, p1, bc, bs):
    mn = bc - bs/2.0
    mx = bc + bs/2.0
    d = p1 - p0
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if p0[i] < mn[i] or p0[i] > mx[i]:
                return False
        else:
            ood = 1.0 / d[i]
            t1 = (mn[i] - p0[i]) * ood
            t2 = (mx[i] - p0[i]) * ood
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False
    return True


def seg_vs_cyl(p0, p1, cc, radius, height):
    p0 = p0 - cc
    p1 = p1 - cc
    d = p1 - p0
    a = d[0]**2 + d[1]**2
    b = 2 * (p0[0]*d[0] + p0[1]*d[1])
    c = p0[0]**2 + p0[1]**2 - radius**2
    if abs(a) < 1e-9:
        return False
    disc = b*b - 4*a*c
    if disc < 0:
        return False
    sq = math.sqrt(disc)
    for t in [(-b - sq)/(2*a), (-b + sq)/(2*a)]:
        if 0 <= t <= 1:
            z = p0[2] + t*d[2]
            if 0 <= z <= height:
                return True
    return False


def pose_collides(pts, chassis_pos, obstacles=()):
    box = get_deadzone_box(chassis_pos)
    cyl = get_deadzone_cyl(chassis_pos)
    for i in range(2, len(pts)-1):
        a, b = pts[i], pts[i+1]
        if seg_vs_box(a, b, box["center"], box["size"]):
            return True
        if seg_vs_cyl(a, b, cyl["center"], cyl["radius"], cyl["height"]):
            return True
        for obs in obstacles:
            if seg_vs_box(a, b, obs["center"], obs["size"]):
                return True
    return False


def path_collides(ang_a, ang_b, chassis_pos, obstacles):
    for k in range(1, PATH_CHECK_STEPS+1):
        t = k / PATH_CHECK_STEPS
        interp = [ang_a[j] + t*(ang_b[j] - ang_a[j]) for j in range(4)]
        pts = forward_kinematics_world(*interp, chassis_pos)
        if pose_collides(pts, chassis_pos, obstacles):
            return True
    return False


def solve_ik_wrist_base(x, y, z, elbow_up=True):
    yaw = math.atan2(y, x)
    r = math.sqrt(x*x + y*y)
    z_eff = z - BASE_HEIGHT
    d2 = r*r + z_eff*z_eff
    d = math.sqrt(d2)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None
    cos_e = clamp((d2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    elbow = math.acos(cos_e)
    if elbow_up:
        elbow = -elbow
    phi = math.atan2(z_eff, r)
    shoulder = phi - math.atan2(L2*math.sin(elbow), L1 + L2*math.cos(elbow))
    return yaw, shoulder, elbow


def forward_kinematics_world(yaw, shoulder, elbow, wrist, chassis_pos):
    am = get_arm_mount(chassis_pos)
    p0 = am.copy()
    pS = p0 + np.array([0., 0., BASE_HEIGHT])
    p1 = pS + np.array([
        L1*math.cos(shoulder)*math.cos(yaw),
        L1*math.cos(shoulder)*math.sin(yaw),
        L1*math.sin(shoulder)
    ])
    t1 = shoulder + elbow
    p2 = p1 + np.array([
        L2*math.cos(t1)*math.cos(yaw),
        L2*math.cos(t1)*math.sin(yaw),
        L2*math.sin(t1)
    ])
    t2 = t1 + wrist
    p3 = p2 + np.array([
        L3*math.cos(t2)*math.cos(yaw),
        L3*math.cos(t2)*math.sin(yaw),
        L3*math.sin(t2)
    ])
    return np.vstack([p0, pS, p1, p2, p3])


def make_finger_polyline(hinge, tdir, inward_dir, close_angle):
    fdir = unit(math.cos(close_angle)*tdir + math.sin(close_angle)*inward_dir)
    tip = hinge + FINGER_LEN*fdir
    cd = np.cross(fdir, tdir)
    if np.linalg.norm(cd) < 1e-6:
        cd = np.cross(fdir, np.array([0., 0., 1.]))
    cd = unit(cd)
    ow = -inward_dir
    cap = []
    for ang in np.linspace(-math.pi/2, math.pi/2, TIP_ARC_PTS):
        cap.append(tip + TIP_ARC_RADIUS*(math.cos(ang)*cd + 0.25*math.sin(ang)*ow))
    return np.vstack([hinge, tip, np.array(cap)])


class ServoController:
    def __init__(self, ser):
        self.ser = ser

        self.current_positions = [90, 0, 180, 90, 180]
        self.target_positions  = [90, 0, 180, 90, 180]

        self.step_sizes = [
            1.0,   # base
            1.2,   # shoulder
            0.7,   # elbow
            1.5,   # wrist
            2.0    # claw
        ]

        self.transition_delays = [
            0.020,  # base
            0.010,  # shoulder
            0.024,  # elbow
            0.010,  # wrist
            0.012   # claw
        ]

        now = time.time()
        self.last_update_times = [now, now, now, now, now]
        self.last_sent = None

    def clamp(self, v, lo=0, hi=180):
        return max(lo, min(hi, v))

    def send_positions(self):
        msg_list = [int(round(self.clamp(p))) for p in self.current_positions]
        msg = ",".join(str(v) for v in msg_list) + "\n"

        if msg == self.last_sent:
            return

        self.ser.write(msg.encode("utf-8"))
        self.last_sent = msg

        reply = self.ser.readline().decode(errors="ignore").strip()
        if reply:
            print("Arduino:", reply)

    def set_positions_immediate(self, positions):
        if len(positions) != 5:
            return

        vals = [self.clamp(int(round(p))) for p in positions]
        self.current_positions = vals[:]
        self.target_positions = vals[:]
        self.send_positions()

    def set_targets(self, positions):
        if len(positions) != 5:
            return
        self.target_positions = [self.clamp(int(round(p))) for p in positions]

    def smooth_transition(self):
        current_time = time.time()
        moved = False

        for i in range(5):
            if current_time - self.last_update_times[i] >= self.transition_delays[i]:
                current = self.current_positions[i]
                target  = self.target_positions[i]
                step    = self.step_sizes[i]

                if current < target:
                    self.current_positions[i] += min(step, target - current)
                    moved = True
                elif current > target:
                    self.current_positions[i] -= min(step, current - target)
                    moved = True

                self.last_update_times[i] = current_time

        if moved:
            self.send_positions()


class IKWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL Arm IK  —  Calibrated Servo Mapping  |  MG996R PWM")

        root = QVBoxLayout()
        self.setLayout(root)

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=800, elevation=18, azimuth=40)
        root.addWidget(self.view, stretch=1)
        grid = gl.GLGridItem()
        grid.scale(50, 50, 1)
        self.view.addItem(grid)

        self.chassis_pos        = np.array([0., 0.], dtype=float)
        self.chassis_target_pos = np.array([0., 0.], dtype=float)
        self.chassis_speed      = 0.05

        

        self.via_angles     = None
        self.target_xyz     = np.array([200., 0., CHASSIS["Lz"]+150.], dtype=float)
        self.grip = self.grip_target = 1.0

        self.arm_total_scale = 2.0

        self.sequence = [
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 1.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 3.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 3.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 3.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 3.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 3.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 50), "wait": 0.0, "speed": 5.0},
            {"xyz": (200, 0, CHASSIS["Lz"] + 5),  "wait": 0.0, "speed": 1.0},
        ]

        self.sequence_enabled = False
        self.sequence_index = 0
        self.sequence_next_time = 0.0

        self.state = "IDLE"
        self.obstacles = []

        sol = self._best_ik(*HOME_XYZ, 0.0, True, self.chassis_pos)
        if sol:
            self.current_angles = sol
            self.target_angles = sol

        self.arm_plot    = gl.GLLinePlotItem(width=5, color=(0.2, 0.7, 1.0, 1.0))
        self.target_plot = gl.GLScatterPlotItem(size=14, color=(1.0, 0.25, 0.25, 1.0))
        self.mount_plot  = gl.GLScatterPlotItem(size=8, color=(1.0, 1.0, 0.0, 1.0))
        self.finger_a    = gl.GLLinePlotItem(width=3, color=(0.9, 0.9, 0.9, 1.0))
        self.finger_b    = gl.GLLinePlotItem(width=3, color=(0.9, 0.9, 0.9, 1.0))

        self.chassis_mesh = make_box_mesh(
            [0, 0, CHASSIS["Lz"]/2], (CHASSIS["Lx"], CHASSIS["Ly"], CHASSIS["Lz"]),
            color=(0.35, 0.35, 0.38, 0.75)
        )
        self.view.addItem(self.chassis_mesh)

        self.turret_mesh = make_cylinder_mesh(
            [0, 0, CHASSIS["Lz"]], TURRET["radius"], TURRET["height"],
            slices=40, color=(0.5, 0.5, 0.55, 0.8)
        )
        self.view.addItem(self.turret_mesh)

        self.turret_dz_mesh = make_cylinder_mesh(
            [0, 0, CHASSIS["Lz"]], TURRET["radius"] + CYL_MARGIN, TURRET["height"] + 2*CYL_MARGIN,
            slices=40, color=(1.0, 0.2, 0.2, 0.13)
        )
        self.view.addItem(self.turret_dz_mesh)

        for item in [self.arm_plot, self.target_plot, self.mount_plot, self.finger_a, self.finger_b]:
            self.view.addItem(item)

        self.status_label = QLabel("Status: IDLE")
        self.status_label.setStyleSheet("font-weight:bold; padding:4px; font-size:12px;")
        root.addWidget(self.status_label)

        self.pose_label = QLabel("Chassis Position: X=0.00 mm, Y=0.00 mm")
        self.pose_label.setStyleSheet("font-size:12px; padding:4px;")
        root.addWidget(self.pose_label)

        self.target_pose_label = QLabel("Target Position: X=--.-- mm, Y=--.-- mm")
        self.target_pose_label.setStyleSheet("font-size:12px; padding:4px; color:#074af2;")
        root.addWidget(self.target_pose_label)

        # Serial / Arduino
        self.ser = None
        self.servo_ctrl = None

        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
            time.sleep(2)

            startup = self.ser.readline().decode(errors="ignore").strip()
            print("Arduino:", startup if startup else "connected")

            self.servo_ctrl = ServoController(self.ser)
            #self.servo_ctrl.send_positions()

            self._status("Arduino connected")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            self.ser = None
            self.servo_ctrl = None
            self._status("Arduino not connected")

        joint_frame = QFrame()
        joint_frame.setStyleSheet("background:#1a1a2e; border-radius:4px; padding:2px;")
        jrow = QHBoxLayout(joint_frame)
        jrow.setContentsMargins(6, 4, 6, 4)
        lbl = QLabel("JOINTS")
        lbl.setStyleSheet("color:#aaaaaa; font-size:10px; font-weight:bold; padding-right:8px;")
        jrow.addWidget(lbl)

        self._jlabels = {}
        self._jbars   = {}
        for name in ["yaw", "shoulder", "elbow", "wrist"]:
            col = QVBoxLayout()
            col.setSpacing(1)
            title = QLabel(name.upper())
            title.setStyleSheet("color:#aaaaaa; font-size:10px;")
            val = QLabel("  0.0°")
            val.setStyleSheet("color:#80ff80; font-size:13px; font-weight:bold;")
            lo, hi = JOINT_LIMITS[name]
            range_lbl = QLabel(f"[{math.degrees(lo):.0f}° … {math.degrees(hi):.0f}°]")
            range_lbl.setStyleSheet("color:#666688; font-size:9px;")
            bar = QLabel()
            bar.setFixedHeight(4)
            bar.setStyleSheet("background:#80ff80; border-radius:2px;")
            col.addWidget(title)
            col.addWidget(val)
            col.addWidget(range_lbl)
            col.addWidget(bar)
            self._jlabels[name] = val
            self._jbars[name] = bar
            jrow.addLayout(col)
        root.addWidget(joint_frame)

        pwm_frame = QFrame()
        pwm_frame.setStyleSheet(
            "background:#0d1f0d; border:1px solid #1a4d1a; border-radius:4px; padding:2px;"
        )
        pwm_outer = QVBoxLayout(pwm_frame)
        pwm_outer.setContentsMargins(6, 3, 6, 3)
        pwm_outer.setSpacing(2)

        hdr_row = QHBoxLayout()
        hdr_lbl = QLabel("⚙  MG996R PWM  (50 Hz / 20 ms period)")
        hdr_lbl.setStyleSheet(
            "color:#44cc44; font-size:10px; font-weight:bold; letter-spacing:1px;"
        )
        hdr_row.addWidget(hdr_lbl)
        hdr_row.addStretch(1)

        legend = QLabel(
            f"  500 µs = 0°  │  1500 µs = 90°  │  2500 µs = 180°  │  Period = {MG996R_PERIOD} µs"
        )
        legend.setStyleSheet("color:#336633; font-size:9px;")
        hdr_row.addWidget(legend)
        pwm_outer.addLayout(hdr_row)

        servo_row = QHBoxLayout()
        servo_row.setSpacing(4)
        self._pwm_labels = {}

        for name in ["yaw", "shoulder", "elbow", "wrist"]:
            col_frame = QFrame()
            col_frame.setStyleSheet(
                "background:#0a2a0a; border:1px solid #1e5c1e; border-radius:3px;"
            )
            col = QVBoxLayout(col_frame)
            col.setContentsMargins(5, 3, 5, 3)
            col.setSpacing(1)

            name_lbl = QLabel(name.upper())
            name_lbl.setStyleSheet(
                "color:#55aa55; font-size:9px; font-weight:bold; letter-spacing:1px;"
            )
            col.addWidget(name_lbl)

            pw_val = QLabel("1500 µs")
            pw_val.setStyleSheet(
                "color:#00ff88; font-size:14px; font-weight:bold; font-family:monospace;"
            )
            col.addWidget(pw_val)

            duty_val = QLabel("7.50 %")
            duty_val.setStyleSheet("color:#55cc77; font-size:10px; font-family:monospace;")
            col.addWidget(duty_val)

            pwm255_val = QLabel("PWM:    0")
            pwm255_val.setStyleSheet(
                "color:#00ff88; font-size:10px; font-weight:bold; font-family:monospace;"
            )
            col.addWidget(pwm255_val)

            pw_bar = QLabel()
            pw_bar.setFixedHeight(5)
            pw_bar.setStyleSheet("background:#00cc66; border-radius:2px; min-width:50px;")
            col.addWidget(pw_bar)

            self._pwm_labels[name] = (pw_val, duty_val, pwm255_val, pw_bar)
            servo_row.addWidget(col_frame)

        pwm_outer.addLayout(servo_row)
        root.addWidget(pwm_frame)

        r1 = QHBoxLayout()
        self.x_in = QLineEdit("200")
        self.y_in = QLineEdit("0")
        self.z_in = QLineEdit(str(int(CHASSIS["Lz"] + 150)))
        for lbl_text, w in [("Target X:", self.x_in), ("  Y:", self.y_in), ("  Z:", self.z_in)]:
            r1.addWidget(QLabel(lbl_text))
            r1.addWidget(w)

        btn_move = QPushButton("▶  Move Arm")
        btn_move.setStyleSheet("font-weight:bold; padding:4px 12px;")
        btn_move.clicked.connect(self.move_to_target)
        r1.addWidget(btn_move)

        btn_seq = QPushButton("▶ Run Sequence")
        btn_seq.clicked.connect(self.start_sequence)
        r1.addWidget(btn_seq)

        btn_home = QPushButton("🏠  Home")
        btn_home.setStyleSheet("padding:4px 10px;")
        btn_home.setToolTip("Return all joints to their centre (home) position")
        btn_home.clicked.connect(self._go_home)
        r1.addWidget(btn_home)
        root.addLayout(r1)

        r2 = QHBoxLayout()
        r2.addWidget(QLabel("Obstacle:"))
        r2.addWidget(QLabel("X:"))
        self.obs_x_in = QLineEdit("150")
        self.obs_x_in.setMaximumWidth(55)
        r2.addWidget(self.obs_x_in)
        r2.addWidget(QLabel("Y:"))
        self.obs_y_in = QLineEdit("0")
        self.obs_y_in.setMaximumWidth(55)
        r2.addWidget(self.obs_y_in)
        r2.addWidget(QLabel("Z:"))
        self.obs_z_in = QLineEdit(str(int(CHASSIS["Lz"] + 60)))
        self.obs_z_in.setMaximumWidth(55)
        r2.addWidget(self.obs_z_in)

        btn_add = QPushButton("➕  Add Obstacle")
        btn_add.setStyleSheet("font-weight:bold; padding:4px 10px;")
        btn_add.clicked.connect(self._add_obstacle)
        r2.addWidget(btn_add)

        btn_clear = QPushButton("🗑  Clear")
        btn_clear.clicked.connect(self._clear_obstacles)
        r2.addWidget(btn_clear)
        r2.addStretch(1)
        root.addLayout(r2)

        r3 = QHBoxLayout()
        self.cb_elbow_up   = QCheckBox("Prefer Elbow Up")
        self.cb_elbow_up.setChecked(True)
        self.cb_auto_pinch = QCheckBox("Auto Pinch")
        self.cb_auto_pinch.setChecked(True)
        self.cb_level      = QCheckBox("Keep Tool Level")
        self.cb_level.setChecked(True)
        for w in [self.cb_elbow_up, self.cb_auto_pinch, self.cb_level]:
            r3.addWidget(w)
        r3.addStretch(1)
        root.addLayout(r3)

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(20)

        self._sync_chassis()
        self._refresh_draw()

    def get_sim_joint_speed(self, joint_name):
        return self.arm_total_scale * SIM_JOINT_SPEEDS[joint_name]

    def _go_home(self):
        self.move_to_xyz(*HOME_XYZ)
        self._status("🏠 Moving to home XYZ position.")

    def _add_obstacle(self):
        try:
            ox = float(self.obs_x_in.text())
            oy = float(self.obs_y_in.text())
            oz = float(self.obs_z_in.text())
        except Exception:
            self._status("⚠️  Invalid obstacle coordinates.")
            return
        center = np.array([ox, oy, oz], dtype=float)
        size = OBS_HALF * 2.0
        mesh = make_box_mesh(center, size, color=(0.1, 0.45, 1.0, 1.0))
        self.view.addItem(mesh)
        self.obstacles.append({"center": center.copy(), "size": size.copy(), "mesh": mesh})
        self._status(f"Obstacle added at ({ox:.0f}, {oy:.0f}, {oz:.0f}) — press ▶ to replan.")

    def _clear_obstacles(self):
        for o in self.obstacles:
            self.view.removeItem(o["mesh"])
        self.obstacles.clear()
        self._status("Obstacles cleared.")

    def _sync_chassis(self):
        T = QMatrix4x4()
        T.translate(float(self.chassis_pos[0]), float(self.chassis_pos[1]), 0.)
        for m in [self.chassis_mesh, self.turret_mesh, self.turret_dz_mesh]:
            m.setTransform(T)

    def _try_ik(self, tx_w, ty_w, tz_w, phi, elbow_up, cp):
        am = get_arm_mount(cp)
        tx, ty, tz = tx_w - am[0], ty_w - am[1], tz_w - am[2]
        yaw0 = math.atan2(ty, tx)
        rxy = (L3 + CLAW_CENTER_OFFSET) * math.cos(phi)
        rz  = (L3 + CLAW_CENTER_OFFSET) * math.sin(phi)
        sol = solve_ik_wrist_base(
            tx - rxy * math.cos(yaw0),
            ty - rxy * math.sin(yaw0),
            tz - rz,
            elbow_up
        )
        if sol is None:
            return None
        yaw, shoulder, elbow = sol
        wrist = phi - (shoulder + elbow)
        if not within_limits(yaw, shoulder, elbow, wrist):
            return None
        pts = forward_kinematics_world(yaw, shoulder, elbow, wrist, cp)
        if pose_collides(pts, cp, self.obstacles):
            return None
        return [yaw, shoulder, elbow, wrist]

    def _best_ik(self, tx_w, ty_w, tz_w, phi, prefer_up, cp):
        for eu in [prefer_up, not prefer_up]:
            sol = self._try_ik(tx_w, ty_w, tz_w, phi, eu, cp)
            if sol is not None:
                return sol
        return None

    def _find_via(self, target_ang, cp):
        jl = JOINT_LIMITS
        best_via = None
        best_cost = float("inf")
        for yaw in np.linspace(jl["yaw"][0], jl["yaw"][1], VIA_YAW_N):
            for sh in np.linspace(jl["shoulder"][0], jl["shoulder"][1], VIA_SHOULDER_N):
                for el in np.linspace(jl["elbow"][0], jl["elbow"][1], VIA_ELBOW_N):
                    wr = -(sh + el)
                    if not within_limits(yaw, sh, el, wr):
                        continue
                    pts = forward_kinematics_world(yaw, sh, el, wr, cp)
                    if pose_collides(pts, cp, self.obstacles):
                        continue
                    via = [yaw, sh, el, wr]
                    if path_collides(self.current_angles, via, cp, self.obstacles):
                        continue
                    if path_collides(via, target_ang, cp, self.obstacles):
                        continue
                    cost = (
                        sum(abs(via[i] - self.current_angles[i]) for i in range(4)) +
                        sum(abs(via[i] - target_ang[i]) for i in range(4))
                    )
                    if cost < best_cost:
                        best_cost = cost
                        best_via = via
        return best_via

    def _find_reposition(self, tx_w, ty_w, tz_w, phi, prefer_up):
        best_sol = None
        best_cp = None
        best_d = float("inf")
        z_rel = tz_w - (CHASSIS["Lz"] + BASE_HEIGHT)
        z_rel = clamp(z_rel, -(L1 + L2)*0.95, (L1 + L2)*0.95)
        horiz_max = math.sqrt(max(0, (L1 + L2)**2 - z_rel**2))
        r_lo = max(100.0, horiz_max * 0.55)
        r_hi = min(1200.0, horiz_max * 1.05 + 200.0)

        for ret in np.linspace(r_lo, r_hi, 28):
            for ang_deg in np.linspace(0, 360, 36, endpoint=False):
                ang = math.radians(ang_deg)
                cp = np.array([tx_w + ret*math.cos(ang), ty_w + ret*math.sin(ang)])
                sol = self._best_ik(tx_w, ty_w, tz_w, phi, prefer_up, cp)
                if sol is not None:
                    if path_collides(self.current_angles, sol, cp, self.obstacles):
                        continue
                    d = float(np.linalg.norm(cp - self.chassis_pos))
                    if d < best_d:
                        best_d = d
                        best_sol = sol
                        best_cp = cp.copy()

        return best_cp, best_sol
    
    def start_sequence(self):
        self.sequence_enabled = True
        self.sequence_index = 0
        self.sequence_next_time = time.time()

    def stop_sequence(self):
        self.sequence_enabled = False

    def move_to_xyz(self, x, y, z):
        self.target_xyz = np.array([x, y, z], dtype=float)
        self.target_plot.setData(pos=np.array([[x, y, z]]))

        phi = 0.0
        pref_up = self.cb_elbow_up.isChecked()
        cp = self.chassis_pos

        sol = self._best_ik(x, y, z, phi, pref_up, cp)
        if sol is not None:
            if not path_collides(self.current_angles, sol, cp, self.obstacles):
                self._commit(sol, None, None)
                self._status(f"Direct path — moving arm to ({x:.0f}, {y:.0f}, {z:.0f}).")
                return True

            self._status("Path sweeps obstacle — searching detour…")
            via = self._find_via(sol, cp)
            if via is not None:
                self._commit(sol, via, None)
                self._status(f"Detouring around obstacle to ({x:.0f}, {y:.0f}, {z:.0f}).")
                return True

        self._status("Searching for reposition…")
        new_cp, new_sol = self._find_reposition(x, y, z, phi, pref_up)
        if new_sol is not None:
            if not path_collides(self.current_angles, new_sol, new_cp, self.obstacles):
                self._commit(new_sol, None, new_cp)
                return True

            via = self._find_via(new_sol, new_cp)
            if via is not None:
                self._commit(new_sol, via, new_cp)
                return True

        self._status(f"❌ No valid path found for ({x:.0f}, {y:.0f}, {z:.0f}).")
        return False

    def move_to_target(self):
        try:
            x = float(self.x_in.text())
            y = float(self.y_in.text())
            z = float(self.z_in.text())
        except Exception:
            self._status("Invalid target values.")
            return

        self.move_to_xyz(x, y, z)

    def _commit(self, target_ang, via_ang, new_cp):
        self.target_angles = target_ang
        self.via_angles = via_ang
        if new_cp is not None:
            self.chassis_target_pos = new_cp
            self.state = "REPOSITIONING"
            self.target_pose_label.setText(
                f"Target Position: X={self.chassis_target_pos[0]:.2f} mm, "
                f"Y={self.chassis_target_pos[1]:.2f} mm"
            )
        else:
            self.state = "ARM_TO_VIA" if via_ang is not None else "ARM_TO_TARGET"

    def _send_sim_to_arm(self):
        if self.servo_ctrl is None:
            return

        servos = current_angles_to_servos(
            self.current_angles,
            grip_value=self.grip
        )

        print(
            "SIM(deg)=",
            [round(math.degrees(a), 1) for a in self.current_angles],
            " -> SERVO=",
            servos
        )

        self.servo_ctrl.set_positions_immediate(servos)

    def _tick(self):
        if self.state == "REPOSITIONING":
            diff = self.chassis_target_pos - self.chassis_pos
            self.chassis_pos += diff * self.chassis_speed
            self._sync_chassis()
            if np.linalg.norm(diff) < 2.0:
                self.chassis_pos = self.chassis_target_pos.copy()
                self.state = "ARM_TO_VIA" if self.via_angles is not None else "ARM_TO_TARGET"
                self._status("Chassis arrived — extending arm.")

        elif self.state == "ARM_TO_VIA":
            if self._smooth_to(self.via_angles) < 0.001:
                self.via_angles = None
                self.state = "ARM_TO_TARGET"
                self._status("Waypoint reached — continuing to target.")

        elif self.state == "ARM_TO_TARGET":
            if self._smooth_to(self.target_angles) < 0.001:
                self.state = "IDLE"
                self._status("✅ Target reached.")

        if self.sequence_enabled and self.state == "IDLE":
            now = time.time()

            if self.sequence_index < len(self.sequence) and now >= self.sequence_next_time:
                wp = self.sequence[self.sequence_index]

                # ===== ADD THIS PART HERE =====
                if "speed" in wp:
                    self.arm_total_scale = float(wp["speed"])
                # =============================

                x, y, z = wp["xyz"]
                ok = self.move_to_xyz(x, y, z)

                if ok:
                    self.sequence_next_time = now + wp["wait"]
                    self.sequence_index += 1
                else:
                    self.sequence_next_time = now + 1.0
                    self.sequence_index += 1

            elif self.sequence_index >= len(self.sequence):
                self.sequence_enabled = False
                self._status("Sequence complete.")
            
        self._refresh_draw()
        self._send_sim_to_arm()

    def _refresh_draw(self):
        yaw, sh, el, wr = self.current_angles
        pts = forward_kinematics_world(yaw, sh, el, wr, self.chassis_pos)
        col = pose_collides(pts, self.chassis_pos, self.obstacles)
        self.arm_plot.setData(
            pos=pts,
            color=(1.0, 0.15, 0.15, 1.0) if col else (0.2, 0.75, 1.0, 1.0)
        )
        self.mount_plot.setData(pos=np.array([get_arm_mount(self.chassis_pos)]))

        tp = sh + el + wr
        tdir = unit(np.array([
            math.cos(tp) * math.cos(yaw),
            math.cos(tp) * math.sin(yaw),
            math.sin(tp)
        ], dtype=float))
        mount = pts[-1]
        cc = mount + CLAW_CENTER_OFFSET * tdir

        if self.cb_auto_pinch.isChecked():
            self.grip_target = 0.0 if np.linalg.norm(cc - self.target_xyz) <= PINCH_DISTANCE_MM else 1.0
        else:
            self.grip_target = 1.0

        self.grip += (self.grip_target - self.grip) * GRIP_SMOOTH
        sdir = unit(np.array([-math.sin(yaw), math.cos(yaw), 0.]))
        hs = 0.5 * CLAW_HINGE_SEP
        ha = mount + hs * sdir
        hb = mount - hs * sdir
        ca = (1.0 - self.grip) * math.asin(max(0.0, min(1.0, hs / max(1e-6, FINGER_LEN))))
        self.finger_a.setData(pos=make_finger_polyline(ha, tdir, -sdir, ca))
        self.finger_b.setData(pos=make_finger_polyline(hb, tdir, +sdir, ca))

        names = ["yaw", "shoulder", "elbow", "wrist"]
        values = [yaw, sh, el, wr]
        for name, val in zip(names, values):
            deg = math.degrees(val)
            fraction = joint_usage_fraction(name, val)
            style = limit_color_style(fraction)
            self._jlabels[name].setText(f"{deg:+.1f}°")
            self._jlabels[name].setStyleSheet(style + "font-size:13px;")
            bar_pct = int(fraction * 100)
            bar_color = "#ff3030" if fraction >= WARN_RED else "#ffcc00" if fraction >= WARN_YELLOW else "#80ff80"
            self._jbars[name].setStyleSheet(
                f"background:{bar_color}; border-radius:2px; "
                f"min-width:{bar_pct}px; max-width:{bar_pct}px;"
            )

        BAR_MAX_PX = 120
        for name, val in zip(names, values):
            pw_us, duty = angle_to_pwm(name, val)
            pwm255 = angle_to_pwm255(name, val)

            fraction = joint_usage_fraction(name, val)
            if fraction >= WARN_RED:
                val_color = "#ff4444"
                bar_color = "#cc2222"
            elif fraction >= WARN_YELLOW:
                val_color = "#ffcc00"
                bar_color = "#cc9900"
            else:
                val_color = "#00ff88"
                bar_color = "#00cc66"

            norm = (pw_us - MG996R_PW_MIN) / (MG996R_PW_MAX - MG996R_PW_MIN)
            bar_px = int(clamp(norm, 0.0, 1.0) * BAR_MAX_PX)

            pw_lbl, duty_lbl, pwm255_lbl, pw_bar = self._pwm_labels[name]

            pw_lbl.setText(f"{pw_us:.0f} µs")
            pw_lbl.setStyleSheet(
                f"color:{val_color}; font-size:14px; font-weight:bold; font-family:monospace;"
            )

            duty_lbl.setText(f"{duty:.2f} %")
            duty_lbl.setStyleSheet(
                f"color:{val_color}; font-size:10px; font-family:monospace;"
            )

            pwm255_lbl.setText(f"PWM: {pwm255:+d}")
            pwm255_lbl.setStyleSheet(
                f"color:{val_color}; font-size:10px; font-weight:bold; font-family:monospace;"
            )

            pw_bar.setStyleSheet(
                f"background:{bar_color}; border-radius:2px; "
                f"min-width:{bar_px}px; max-width:{bar_px}px; min-height:5px;"
            )

        self.pose_label.setText(
            f"Chassis Position: X={self.chassis_pos[0]:.2f} mm, "
            f"Y={self.chassis_pos[1]:.2f} mm"
        )

    def _smooth_to(self, target):
        err = 0.0

        joint_names = ["yaw", "shoulder", "elbow", "wrist"]

        for i, name in enumerate(joint_names):
            d = target[i] - self.current_angles[i]
            speed = self.get_sim_joint_speed(name)
            self.current_angles[i] += d * speed
            err += abs(d)

        return err

    def _status(self, msg):
        self.status_label.setText(f"Status: {msg}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = IKWindow()
    win.resize(1050, 820)
    win.show()
    sys.exit(app.exec_())