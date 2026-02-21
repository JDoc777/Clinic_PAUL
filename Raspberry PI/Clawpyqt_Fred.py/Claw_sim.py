import sys
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl

# ============================================================
# CONFIG: Robot chassis + arm mounting
# ============================================================

CHASSIS = {
    "Lx": 304.8,
    "Ly": 245.0,
    "Lz": 120.0,
}

ARM_MOUNT_WORLD = np.array([
    0.0,
    0.0,
    CHASSIS["Lz"]  # top surface
], dtype=float)

TURRET = {
    "radius": 45.0,
    "height": 18.0
}

BASE_HEIGHT = 20.0               # shoulder pivot height above yaw axis
BASE_TO_SHOULDER_OFFSET = 0.0    # horizontal offset from yaw axis to shoulder pivot (optional)

# ============================================================
# ARM PARAMETERS (mm)
# ============================================================
L1 = 152.4
L2 = 228.6
L3 = 40.0

PHI_TOOL = 0.0  # tool pitch in world frame (radians), 0 = level

# ============================================================
# CLAW PARAMETERS (mm)
# ============================================================
CLAW_HINGE_SEP = 16.0
FINGER_LEN     = 20.0
TIP_ARC_RADIUS = 5.0
TIP_ARC_PTS    = 10

PINCH_DISTANCE_MM = 12.0
GRIP_SMOOTH = 0.12

CLAW_CENTER_OFFSET = FINGER_LEN  # along tool direction

# ============================================================
# JOINT LIMITS (radians)  <<< TUNE THESE FOR YOUR ARM >>>
# ============================================================
JOINT_LIMITS = {
    "yaw":      (-math.pi, math.pi),
    "shoulder": (math.radians(-120), math.radians(90)),
    "elbow":    (math.radians(-150), math.radians(0)),   # common: elbow bends negative in your convention
    "wrist":    (math.radians(-120), math.radians(120)),
}

# ============================================================
# DEADZONES (collision volumes)  <<< TUNE MARGINS HERE >>>
# ============================================================
# Box deadzone: the chassis itself (add margin if you want)
BOX_MARGIN = 3.0  # mm safety margin
DEADZONE_BOX = {
    "center": np.array([0.0, 0.0, CHASSIS["Lz"] / 2.0], dtype=float),
    "size":   np.array([CHASSIS["Lx"] + 2*BOX_MARGIN,
                        CHASSIS["Ly"] + 2*BOX_MARGIN,
                        CHASSIS["Lz"] + 2*BOX_MARGIN], dtype=float)
}

# Cylinder deadzone: turret area (add margin)
CYL_MARGIN = 8.0  # mm safety margin
DEADZONE_CYL = {
    "center": np.array([ARM_MOUNT_WORLD[0], ARM_MOUNT_WORLD[1], CHASSIS["Lz"]], dtype=float),
    "radius": TURRET["radius"] + CYL_MARGIN,
    "height": TURRET["height"] + 2*CYL_MARGIN
}

# ============================================================
# Utility
# ============================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def unit(v):
    n = float(np.linalg.norm(v))
    return v / (n + 1e-9)

def within_limits(yaw, shoulder, elbow, wrist):
    if not (JOINT_LIMITS["yaw"][0] <= yaw <= JOINT_LIMITS["yaw"][1]):
        return False
    if not (JOINT_LIMITS["shoulder"][0] <= shoulder <= JOINT_LIMITS["shoulder"][1]):
        return False
    if not (JOINT_LIMITS["elbow"][0] <= elbow <= JOINT_LIMITS["elbow"][1]):
        return False
    if not (JOINT_LIMITS["wrist"][0] <= wrist <= JOINT_LIMITS["wrist"][1]):
        return False
    return True

# ============================================================
# Mesh helpers (chassis box + turret)
# ============================================================
def make_box_mesh(center, size_xyz, color=(0.3, 0.3, 0.35, 0.6)):
    sx, sy, sz = size_xyz
    cx, cy, cz = center

    x0, x1 = cx - sx/2, cx + sx/2
    y0, y1 = cy - sy/2, cy + sy/2
    z0, z1 = cz - sz/2, cz + sz/2

    verts = np.array([
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1],
    ], dtype=float)

    faces = np.array([
        [0, 1, 2], [0, 2, 3],
        [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0],
    ], dtype=int)

    md = gl.MeshData(vertexes=verts, faces=faces)
    item = gl.GLMeshItem(meshdata=md, smooth=False, drawFaces=True, drawEdges=True,
                         edgeColor=(0.1, 0.1, 0.12, 0.7))
    item.setColor(color)
    return item

def make_cylinder_mesh(center, radius, height, slices=36, color=(0.45, 0.45, 0.5, 0.7)):
    cx, cy, cz = center
    z0 = cz
    z1 = cz + height

    angles = np.linspace(0, 2*np.pi, slices, endpoint=False)
    circle0 = np.stack([cx + radius*np.cos(angles), cy + radius*np.sin(angles), np.full_like(angles, z0)], axis=1)
    circle1 = np.stack([cx + radius*np.cos(angles), cy + radius*np.sin(angles), np.full_like(angles, z1)], axis=1)

    verts = np.vstack([circle0, circle1, np.array([[cx, cy, z0], [cx, cy, z1]])])
    idx_center0 = 2*slices
    idx_center1 = 2*slices + 1

    faces = []
    for i in range(slices):
        j = (i + 1) % slices
        faces.append([i, j, slices + j])
        faces.append([i, slices + j, slices + i])
    for i in range(slices):
        j = (i + 1) % slices
        faces.append([idx_center0, j, i])
    for i in range(slices):
        j = (i + 1) % slices
        faces.append([idx_center1, slices + i, slices + j])

    faces = np.array(faces, dtype=int)
    md = gl.MeshData(vertexes=verts, faces=faces)
    item = gl.GLMeshItem(meshdata=md, smooth=True, drawFaces=True, drawEdges=False)
    item.setColor(color)
    return item

# ============================================================
# Collision: segment vs AABB (box), segment vs vertical cylinder
# ============================================================
def segment_intersects_box(p0, p1, box_center, box_size):
    """Slab method for segment-AABB intersection."""
    min_corner = box_center - box_size / 2.0
    max_corner = box_center + box_size / 2.0

    d = p1 - p0
    tmin = 0.0
    tmax = 1.0

    for i in range(3):
        if abs(d[i]) < 1e-9:
            if p0[i] < min_corner[i] or p0[i] > max_corner[i]:
                return False
        else:
            ood = 1.0 / d[i]
            t1 = (min_corner[i] - p0[i]) * ood
            t2 = (max_corner[i] - p0[i]) * ood
            t1, t2 = (t1, t2) if t1 <= t2 else (t2, t1)
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return False

    return True

def segment_intersects_cylinder(p0, p1, cyl_center, radius, height):
    """
    Vertical cylinder aligned with +z.
    cyl_center = base center (x,y,z_base)
    height extends from z_base to z_base+height.
    """
    # shift
    p0 = p0 - cyl_center
    p1 = p1 - cyl_center
    d = p1 - p0

    a = d[0]**2 + d[1]**2
    b = 2.0 * (p0[0]*d[0] + p0[1]*d[1])
    c = p0[0]**2 + p0[1]**2 - radius**2

    if abs(a) < 1e-9:
        return False

    disc = b*b - 4*a*c
    if disc < 0:
        return False

    sqrt_disc = math.sqrt(disc)
    t_candidates = [(-b - sqrt_disc) / (2*a), (-b + sqrt_disc) / (2*a)]

    for t in t_candidates:
        if 0.0 <= t <= 1.0:
            z = p0[2] + t * d[2]
            if 0.0 <= z <= height:
                return True

    return False

def pose_collides(points_world):
    """
    Check collisions for link segments.
    points_world = [p0(mount), pS(shoulder pivot), p1(elbow), p2(wrist base), p3(claw mount)]
    We check segments: pS->p1, p1->p2, p2->p3
    """
    # indices 1..4 are arm chain points, check consecutive pairs
    for i in range(1, len(points_world)-1):
        a = points_world[i]
        b = points_world[i+1]

        if segment_intersects_box(a, b, DEADZONE_BOX["center"], DEADZONE_BOX["size"]):
            return True

        if segment_intersects_cylinder(a, b, DEADZONE_CYL["center"], DEADZONE_CYL["radius"], DEADZONE_CYL["height"]):
            return True

    return False

# ============================================================
# IK + FK
# ============================================================
def solve_ik_wrist_base(x, y, z, elbow_up=True):
    """
    Solve IK to reach point (x,y,z) at the end of L2 (wrist base),
    in frame where yaw axis origin is (0,0,0) and shoulder pivot is at (0,0,BASE_HEIGHT).
    """
    yaw = math.atan2(y, x)
    r = math.sqrt(x*x + y*y)

    z_eff = z - BASE_HEIGHT

    d2 = r*r + z_eff*z_eff
    d = math.sqrt(d2)

    if d > (L1 + L2) or d < abs(L1 - L2):
        return None

    cos_elbow = clamp((d2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    elbow = math.acos(cos_elbow)
    if elbow_up:
        elbow = -elbow

    phi = math.atan2(z_eff, r)
    k1 = L1 + L2 * math.cos(elbow)
    k2 = L2 * math.sin(elbow)
    shoulder = phi - math.atan2(k2, k1)

    return yaw, shoulder, elbow

def forward_kinematics_world(yaw, shoulder, elbow, wrist):
    """
    Returns points in WORLD coords:
    p0 = mount (yaw origin)
    pS = shoulder pivot
    p1 = elbow
    p2 = wrist base
    p3 = claw mount
    """
    p0 = ARM_MOUNT_WORLD.copy()

    # shoulder pivot
    pS = p0 + np.array([0.0, 0.0, BASE_HEIGHT], dtype=float)

    # optional horizontal offset in yaw-forward direction
    off = BASE_TO_SHOULDER_OFFSET
    if abs(off) > 1e-9:
        pS = pS + np.array([math.cos(yaw)*off, math.sin(yaw)*off, 0.0], dtype=float)

    # elbow
    p1 = pS + np.array([
        L1 * math.cos(shoulder) * math.cos(yaw),
        L1 * math.cos(shoulder) * math.sin(yaw),
        L1 * math.sin(shoulder)
    ], dtype=float)

    # wrist base
    total_1 = shoulder + elbow
    p2 = p1 + np.array([
        L2 * math.cos(total_1) * math.cos(yaw),
        L2 * math.cos(total_1) * math.sin(yaw),
        L2 * math.sin(total_1)
    ], dtype=float)

    # claw mount
    total_2 = total_1 + wrist
    p3 = p2 + np.array([
        L3 * math.cos(total_2) * math.cos(yaw),
        L3 * math.cos(total_2) * math.sin(yaw),
        L3 * math.sin(total_2)
    ], dtype=float)

    return np.vstack([p0, pS, p1, p2, p3])

def make_finger_polyline(hinge, tdir, inward_dir, close_angle):
    a = close_angle
    fdir = unit(math.cos(a) * tdir + math.sin(a) * inward_dir)
    tip = hinge + FINGER_LEN * fdir

    cap = []
    cap_dir = np.cross(fdir, tdir)
    if np.linalg.norm(cap_dir) < 1e-6:
        cap_dir = np.cross(fdir, np.array([0.0, 0.0, 1.0]))
    cap_dir = unit(cap_dir)

    outward = -inward_dir
    angles = np.linspace(-math.pi / 2, math.pi / 2, TIP_ARC_PTS)
    for ang in angles:
        cap.append(tip + TIP_ARC_RADIUS * (math.cos(ang) * cap_dir + 0.25 * math.sin(ang) * outward))

    return np.vstack([hinge, tip, np.array(cap)])

# ============================================================
# Main GUI
# ============================================================
class IKWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL Arm + Claw IK Simulator (with Deadzones)")

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=650, elevation=18, azimuth=40)
        layout.addWidget(self.view)

        grid = gl.GLGridItem()
        grid.scale(50, 50, 1)
        self.view.addItem(grid)

        # chassis
        chassis_center = np.array([0.0, 0.0, CHASSIS["Lz"]/2], dtype=float)
        self.chassis = make_box_mesh(chassis_center, (CHASSIS["Lx"], CHASSIS["Ly"], CHASSIS["Lz"]),
                                     color=(0.35, 0.35, 0.38, 0.75))
        self.view.addItem(self.chassis)

        # turret visual (actual turret)
        turret_center = ARM_MOUNT_WORLD.copy()
        turret_center[2] = CHASSIS["Lz"]
        self.turret = make_cylinder_mesh(turret_center, TURRET["radius"], TURRET["height"],
                                         slices=40, color=(0.5, 0.5, 0.55, 0.8))
        self.view.addItem(self.turret)

        # turret deadzone visual (slightly bigger, transparent)
        self.turret_deadzone = make_cylinder_mesh(DEADZONE_CYL["center"], DEADZONE_CYL["radius"], DEADZONE_CYL["height"],
                                                  slices=40, color=(1.0, 0.2, 0.2, 0.18))
        self.view.addItem(self.turret_deadzone)

        # arm plot (we will recolor it on collision)
        self.arm_plot = gl.GLLinePlotItem(width=5, color=(0.2, 0.7, 1.0, 1.0))
        self.view.addItem(self.arm_plot)

        self.target_plot = gl.GLScatterPlotItem(size=10, color=(1.0, 0.3, 0.3, 1.0))
        self.view.addItem(self.target_plot)

        self.mount_plot = gl.GLScatterPlotItem(size=8, color=(1.0, 1.0, 0.0, 1.0))
        self.view.addItem(self.mount_plot)
        self.mount_plot.setData(pos=np.array([ARM_MOUNT_WORLD]))

        self.finger_a = gl.GLLinePlotItem(width=3, color=(0.9, 0.9, 0.9, 1.0))
        self.finger_b = gl.GLLinePlotItem(width=3, color=(0.9, 0.9, 0.9, 1.0))
        self.view.addItem(self.finger_a)
        self.view.addItem(self.finger_b)

        # inputs
        input_layout = QHBoxLayout()
        self.x_input = QLineEdit("200")
        self.y_input = QLineEdit("0")
        self.z_input = QLineEdit(str(CHASSIS["Lz"] + 80))

        input_layout.addWidget(QLabel("Target X (mm):"))
        input_layout.addWidget(self.x_input)
        input_layout.addWidget(QLabel("Y:"))
        input_layout.addWidget(self.y_input)
        input_layout.addWidget(QLabel("Z:"))
        input_layout.addWidget(self.z_input)

        move_btn = QPushButton("Move")
        move_btn.clicked.connect(self.move_to_target)
        input_layout.addWidget(move_btn)
        layout.addLayout(input_layout)

        # options
        opt_layout = QHBoxLayout()
        self.elbow_up_checkbox = QCheckBox("Prefer Elbow Up")
        self.elbow_up_checkbox.setChecked(True)
        opt_layout.addWidget(self.elbow_up_checkbox)

        self.auto_pinch_checkbox = QCheckBox("Auto Pinch")
        self.auto_pinch_checkbox.setChecked(True)
        opt_layout.addWidget(self.auto_pinch_checkbox)

        self.level_tool_checkbox = QCheckBox("Keep Tool Level (pitch=0)")
        self.level_tool_checkbox.setChecked(True)
        opt_layout.addWidget(self.level_tool_checkbox)

        opt_layout.addStretch(1)
        layout.addLayout(opt_layout)

        # state
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.target_angles  = [0.0, 0.0, 0.0, 0.0]

        self.target_xyz = np.array([200.0, 0.0, CHASSIS["Lz"] + 80.0], dtype=float)

        self.grip = 1.0
        self.grip_target = 1.0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_arm)
        self.timer.start(20)

        self.move_to_target()

    def _try_solution(self, tx, ty, tz, phi_tool, elbow_up):
        """
        Attempt one IK solution (elbow branch), then check:
        - joint limits
        - collisions
        Returns [yaw, shoulder, elbow, wrist] or None.
        """
        # yaw from target direction (relative to mount)
        yaw0 = math.atan2(ty, tx)

        # back off from desired claw-center by (L3 + CLAW_CENTER_OFFSET) along tool direction
        tool_xy = (L3 + CLAW_CENTER_OFFSET) * math.cos(phi_tool)
        tool_z  = (L3 + CLAW_CENTER_OFFSET) * math.sin(phi_tool)

        # wrist-base target (end of L2)
        xw = tx - tool_xy * math.cos(yaw0)
        yw = ty - tool_xy * math.sin(yaw0)
        zw = tz - tool_z

        sol = solve_ik_wrist_base(xw, yw, zw, elbow_up=elbow_up)
        if sol is None:
            return None

        yaw, shoulder, elbow = sol
        wrist = phi_tool - (shoulder + elbow)

        # joint limits
        if not within_limits(yaw, shoulder, elbow, wrist):
            return None

        # collision check
        pts = forward_kinematics_world(yaw, shoulder, elbow, wrist)
        if pose_collides(pts):
            return None

        return [yaw, shoulder, elbow, wrist]

    def move_to_target(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except:
            return

        # reject below chassis top (change if you want it to reach into chassis)
        if z < CHASSIS["Lz"]:
            print("Rejected: target is below chassis top.")
            return

        self.target_xyz = np.array([x, y, z], dtype=float)
        self.target_plot.setData(pos=np.array([[x, y, z]]))

        # desired pitch
        phi_tool = 0.0 if self.level_tool_checkbox.isChecked() else PHI_TOOL

        # convert WORLD target into local coords relative to yaw axis origin
        tx, ty, tz = self.target_xyz - ARM_MOUNT_WORLD

        prefer_elbow_up = self.elbow_up_checkbox.isChecked()

        # Try preferred branch first, then alternate
        candidates = []
        cand1 = self._try_solution(tx, ty, tz, phi_tool, elbow_up=prefer_elbow_up)
        if cand1 is not None:
            candidates.append(cand1)

        cand2 = self._try_solution(tx, ty, tz, phi_tool, elbow_up=not prefer_elbow_up)
        if cand2 is not None:
            candidates.append(cand2)

        if not candidates:
            print("No valid (non-colliding) IK solution found for target.")
            return

        # pick closest to current to avoid flips
        def score(c):
            return sum((c[i] - self.current_angles[i])**2 for i in range(4))

        best = min(candidates, key=score)
        self.target_angles = best

    def update_arm(self):
        # smooth joints
        joint_speed = 0.06
        for i in range(4):
            diff = self.target_angles[i] - self.current_angles[i]
            self.current_angles[i] += diff * joint_speed

        yaw, shoulder, elbow, wrist = self.current_angles

        pts = forward_kinematics_world(yaw, shoulder, elbow, wrist)
        colliding_now = pose_collides(pts)

        # color arm based on collision
        if colliding_now:
            self.arm_plot.setData(pos=pts, color=(1.0, 0.2, 0.2, 1.0))
        else:
            self.arm_plot.setData(pos=pts, color=(0.2, 0.7, 1.0, 1.0))

        mount = pts[-1]

        # tool direction
        tool_pitch = shoulder + elbow + wrist
        tdir = unit(np.array([
            math.cos(tool_pitch) * math.cos(yaw),
            math.cos(tool_pitch) * math.sin(yaw),
            math.sin(tool_pitch)
        ], dtype=float))

        claw_center = mount + CLAW_CENTER_OFFSET * tdir

        # auto pinch based on distance from claw-center to target
        if self.auto_pinch_checkbox.isChecked():
            dist = float(np.linalg.norm(claw_center - self.target_xyz))
            self.grip_target = 0.0 if dist <= PINCH_DISTANCE_MM else 1.0
        else:
            self.grip_target = 1.0

        self.grip += (self.grip_target - self.grip) * GRIP_SMOOTH

        # stable side direction for hinge placement
        sdir = unit(np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float))

        half_sep = 0.5 * CLAW_HINGE_SEP
        hinge_a = mount + (+half_sep) * sdir
        hinge_b = mount + (-half_sep) * sdir

        ratio = max(0.0, min(1.0, half_sep / max(1e-6, FINGER_LEN)))
        max_close = math.asin(ratio)
        close_angle = (1.0 - self.grip) * max_close

        inward_a = -sdir
        inward_b = +sdir

        fa = make_finger_polyline(hinge_a, tdir, inward_a, close_angle)
        fb = make_finger_polyline(hinge_b, tdir, inward_b, close_angle)

        self.finger_a.setData(pos=fa)
        self.finger_b.setData(pos=fb)

# ============================================================
# Run App
# ============================================================
app = QApplication(sys.argv)
window = IKWindow()
window.show()
sys.exit(app.exec_())