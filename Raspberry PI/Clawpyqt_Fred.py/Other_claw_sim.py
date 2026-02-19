import sys
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QCheckBox
)
from PyQt5.QtCore import QTimer
import pyqtgraph.opengl as gl

# --------------------------
# Arm Parameters (mm)
# --------------------------
L1 = 110
L2 = 110
L3 = 40   # wrist/tool link length (wrist base -> claw mount)

# Desired tool pitch in world frame (radians)
PHI_TOOL = 0.0  # 0 = level/horizontal

# --------------------------
# Claw parameters (mm)
# --------------------------
CLAW_HINGE_SEP = 16.0   # distance between hinge points (left/right)
FINGER_LEN     = 20.0   # hinge -> fingertip length

# Tiny rounded cap (optional)
TIP_ARC_RADIUS = 5.0
TIP_ARC_PTS    = 10

# Auto-pinch behavior
PINCH_DISTANCE_MM = 12.0
GRIP_SMOOTH = 0.12

# IMPORTANT:
# We now target the "dead center of the claw" = midpoint between the TWO FINGERTIPS
# When open (close_angle=0), that midpoint is forward by FINGER_LEN from the claw mount.
CLAW_CENTER_OFFSET = FINGER_LEN  # along tool direction


# --------------------------
# IK Solver (2-link arm reaching a point) + elbow branch
# --------------------------
def solve_ik(x, y, z, elbow_up=True):
    theta_yaw = math.atan2(y, x)
    r = math.sqrt(x**2 + y**2)
    d = math.sqrt(r**2 + z**2)

    if d > L1 + L2:
        return None

    cos_elbow = (r**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_elbow = max(-1, min(1, cos_elbow))

    theta_elbow = math.acos(cos_elbow)
    if elbow_up:
        theta_elbow = -theta_elbow

    phi = math.atan2(z, r)
    k1 = L1 + L2 * math.cos(theta_elbow)
    k2 = L2 * math.sin(theta_elbow)
    theta_shoulder = phi - math.atan2(k2, k1)

    return theta_yaw, theta_shoulder, theta_elbow


# --------------------------
# Forward Kinematics (4 DOF)
# Returns points up to claw mount (end of L3)
# --------------------------
def forward_kinematics(yaw, shoulder, elbow, wrist):
    x1 = L1 * math.cos(shoulder) * math.cos(yaw)
    y1 = L1 * math.cos(shoulder) * math.sin(yaw)
    z1 = L1 * math.sin(shoulder)

    total_1 = shoulder + elbow
    x2 = x1 + L2 * math.cos(total_1) * math.cos(yaw)
    y2 = y1 + L2 * math.cos(total_1) * math.sin(yaw)
    z2 = z1 + L2 * math.sin(total_1)

    total_2 = total_1 + wrist
    x3 = x2 + L3 * math.cos(total_2) * math.cos(yaw)
    y3 = y2 + L3 * math.cos(total_2) * math.sin(yaw)
    z3 = z2 + L3 * math.sin(total_2)

    return np.array([
        [0, 0, 0],
        [x1, y1, z1],
        [x2, y2, z2],
        [x3, y3, z3]  # claw mount point (between hinges)
    ])


def unit(v):
    n = float(np.linalg.norm(v))
    return v / (n + 1e-9)


def make_finger_polyline(hinge, tdir, inward_dir, close_angle):
    """
    Straight finger that rotates inward.
    - close_angle=0 => points straight out along tool direction
    - close_angle=max_close => tips meet without overlap
    """
    a = close_angle
    fdir = unit(math.cos(a) * tdir + math.sin(a) * inward_dir)
    tip = hinge + FINGER_LEN * fdir

    # Tiny rounded cap
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


# --------------------------
# Main GUI
# --------------------------
class IKWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("3D IK Claw Control (Target = Claw Center)")

        layout = QVBoxLayout()
        self.setLayout(layout)

        # 3D View
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=400)
        layout.addWidget(self.view)

        grid = gl.GLGridItem()
        grid.scale(20, 20, 1)
        self.view.addItem(grid)

        self.arm_plot = gl.GLLinePlotItem(width=4)
        self.view.addItem(self.arm_plot)

        # Target = desired CLAW CENTER (midpoint between fingertips)
        self.target_plot = gl.GLScatterPlotItem(size=10)
        self.view.addItem(self.target_plot)

        # Optional: show where the claw mount (end of wrist link) is
        self.mount_plot = gl.GLScatterPlotItem(size=6)
        self.view.addItem(self.mount_plot)

        # Curved/hinged claw fingers
        self.finger_a = gl.GLLinePlotItem(width=3)
        self.finger_b = gl.GLLinePlotItem(width=3)
        self.view.addItem(self.finger_a)
        self.view.addItem(self.finger_b)

        # Inputs
        input_layout = QHBoxLayout()
        self.x_input = QLineEdit("150")
        self.y_input = QLineEdit("0")
        self.z_input = QLineEdit("80")

        input_layout.addWidget(QLabel("X:"))
        input_layout.addWidget(self.x_input)
        input_layout.addWidget(QLabel("Y:"))
        input_layout.addWidget(self.y_input)
        input_layout.addWidget(QLabel("Z:"))
        input_layout.addWidget(self.z_input)

        move_btn = QPushButton("Move")
        move_btn.clicked.connect(self.move_to_target)
        input_layout.addWidget(move_btn)
        layout.addLayout(input_layout)

        # Options
        opt_layout = QHBoxLayout()
        self.elbow_up_checkbox = QCheckBox("Elbow Up")
        self.elbow_up_checkbox.setChecked(True)
        opt_layout.addWidget(self.elbow_up_checkbox)

        self.auto_pinch_checkbox = QCheckBox("Auto Pinch")
        self.auto_pinch_checkbox.setChecked(True)
        opt_layout.addWidget(self.auto_pinch_checkbox)

        opt_layout.addStretch(1)
        layout.addLayout(opt_layout)

        # Joint states: yaw, shoulder, elbow, wrist
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.target_angles  = [0.0, 0.0, 0.0, 0.0]

        # Target storage
        self.target_xyz = np.array([150.0, 0.0, 80.0], dtype=float)

        # Grip: 1=open, 0=closed
        self.grip = 1.0
        self.grip_target = 1.0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_arm)
        self.timer.start(20)

        self.move_to_target()

    def move_to_target(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except:
            return

        # This marker is the desired CLAW CENTER (midpoint between fingertips)
        self.target_xyz = np.array([x, y, z], dtype=float)
        self.target_plot.setData(pos=np.array([[x, y, z]]))

        # Yaw direction from target (good enough for this arm)
        yaw_guess = math.atan2(y, x)

        # Back off from desired claw-center by (L3 + CLAW_CENTER_OFFSET) along tool direction
        tool_xy = (L3 + CLAW_CENTER_OFFSET) * math.cos(PHI_TOOL)
        tool_z  = (L3 + CLAW_CENTER_OFFSET) * math.sin(PHI_TOOL)

        # This is the wrist-base target for the 2-link IK (end of L2)
        xw = x - tool_xy * math.cos(yaw_guess)
        yw = y - tool_xy * math.sin(yaw_guess)
        zw = z - tool_z

        elbow_up = self.elbow_up_checkbox.isChecked()
        sol = solve_ik(xw, yw, zw, elbow_up=elbow_up)
        if sol is None:
            print("Target unreachable (after claw-center offset)")
            return

        yaw, shoulder, elbow = sol
        wrist = PHI_TOOL - (shoulder + elbow)  # auto wrist

        self.target_angles = [yaw, shoulder, elbow, wrist]

    def update_arm(self):
        # Smooth joints
        joint_speed = 0.05
        for i in range(4):
            diff = self.target_angles[i] - self.current_angles[i]
            self.current_angles[i] += diff * joint_speed

        yaw, shoulder, elbow, wrist = self.current_angles

        # Draw arm (to claw mount)
        points = forward_kinematics(yaw, shoulder, elbow, wrist)
        self.arm_plot.setData(pos=points)

        # Claw mount point (end of wrist link)
        mount = points[-1]
        self.mount_plot.setData(pos=np.array([mount]))

        # Auto pinch based on DISTANCE FROM CLAW CENTER to target
        tool_pitch = shoulder + elbow + wrist
        tdir = unit(np.array([
            math.cos(tool_pitch) * math.cos(yaw),
            math.cos(tool_pitch) * math.sin(yaw),
            math.sin(tool_pitch)
        ], dtype=float))

        # Current claw-center point (midpoint between fingertips when OPEN is mount + FINGER_LEN*tdir,
        # but we define "claw center" consistently as mount + CLAW_CENTER_OFFSET*tdir)
        claw_center = mount + CLAW_CENTER_OFFSET * tdir

        if self.auto_pinch_checkbox.isChecked():
            dist = float(np.linalg.norm(claw_center - self.target_xyz))
            self.grip_target = 0.0 if dist <= PINCH_DISTANCE_MM else 1.0
        else:
            self.grip_target = 1.0

        self.grip += (self.grip_target - self.grip) * GRIP_SMOOTH

        # Side direction (stable) for hinge locations
        sdir = unit(np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float))

        # Two hinge points attached to the mount (not the target)
        half_sep = 0.5 * CLAW_HINGE_SEP
        hinge_a = mount + (+half_sep) * sdir
        hinge_b = mount + (-half_sep) * sdir

        # Choose max_close so tips just touch (no overlap)
        ratio = max(0.0, min(1.0, half_sep / max(1e-6, FINGER_LEN)))
        max_close = math.asin(ratio)

        # close_angle goes 0 (open) -> max_close (closed)
        close_angle = (1.0 - self.grip) * max_close

        inward_a = -sdir
        inward_b = +sdir

        fa = make_finger_polyline(hinge_a, tdir, inward_a, close_angle)
        fb = make_finger_polyline(hinge_b, tdir, inward_b, close_angle)

        self.finger_a.setData(pos=fa)
        self.finger_b.setData(pos=fb)


# --------------------------
# Run App
# --------------------------
app = QApplication(sys.argv)
window = IKWindow()
window.show()
sys.exit(app.exec_())
