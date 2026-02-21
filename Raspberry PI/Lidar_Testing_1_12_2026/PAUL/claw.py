import sys
import math
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl

PREV_IK = None


# ===================== ARM GEOMETRY =====================
ARM = {
    "L1": 10.0,
    "L2": 10.0,
    "base_height": 5.0
}

# ===================== JOINT LIMITS (math space, degrees) =====================
JOINT_LIMITS = {
    "base": (-180, 180),
    "shoulder": (-120, 90),
    "elbow": (-150, 0),
}

# ===================== SERVO OFFSETS (hardware space) =====================
SERVO_OFFSETS = {
    "base": 90,
    "shoulder": 90,
    "elbow": 90
}

SERVO_LIMITS = {
    "base": (0, 180),
    "shoulder": (0, 180),
    "elbow": (0, 180)
}

SERVO_DIRECTIONS = {
    "base": 1,
    "shoulder": 1,
    "elbow": 1
}

# ===================== UTILS =====================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ===================== REACHABILITY =====================
def classify_reachability(x, y, z, arm):
    L1, L2 = arm["L1"], arm["L2"]
    base_h = arm["base_height"]

    r = math.sqrt(x*x + y*y)
    z_eff = z - base_h
    dist = math.sqrt(r*r + z_eff*z_eff)

    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    if dist > max_reach or dist < min_reach:
        return "unreachable"
    elif dist > 0.92 * max_reach:
        return "near_singularity"
    else:
        return "safe"

# ===================== IK SOLVER (MATH SPACE) =====================
def solve_ik_3d(x, y, z, arm, prev_angles=None):
    L1, L2 = arm["L1"], arm["L2"]
    base_h = arm["base_height"]

    # Base rotation
    theta_base = math.atan2(y, x)

    r = math.sqrt(x*x + y*y)
    z_eff = z - base_h
    dist2 = r*r + z_eff*z_eff
    dist = math.sqrt(dist2)

    if dist > (L1 + L2) or dist < abs(L1 - L2):
        return None

    # Elbow angle (elbow-up branch)
    D = clamp((dist2 - L1*L1 - L2*L2) / (2*L1*L2), -1, 1)
    theta_elbow = -math.acos(D)

    phi = math.atan2(z_eff, r)
    psi = math.atan2(
        L2 * math.sin(theta_elbow),
        L1 + L2 * math.cos(theta_elbow)
    )

    # Generate BOTH shoulder solutions
    solutions = []

    for theta_shoulder in (phi - psi, phi + psi):
        angles = {
            "base": math.degrees(theta_base),
            "shoulder": math.degrees(theta_shoulder),
            "elbow": math.degrees(theta_elbow)
        }

        # Enforce joint limits
        valid = True
        for j in angles:
            lo, hi = JOINT_LIMITS[j]
            if not (lo <= angles[j] <= hi):
                valid = False
                break

        if valid:
            solutions.append(angles)

    if not solutions:
        return None

    # First frame: pick any valid solution
    if prev_angles is None:
        return solutions[0]

    # Continuity: pick closest solution
    def joint_distance(sol):
        return sum((sol[j] - prev_angles[j]) ** 2 for j in sol)

    return min(solutions, key=joint_distance)

# ===================== FORWARD KINEMATICS =====================
def forward_kinematics(angles, arm):
    L1, L2 = arm["L1"], arm["L2"]
    base_h = arm["base_height"]

    tb = math.radians(angles["base"])
    ts = math.radians(angles["shoulder"])
    te = math.radians(angles["elbow"])

    p0 = np.array([0, 0, base_h])

    p1 = np.array([
        math.cos(tb) * L1 * math.cos(ts),
        math.sin(tb) * L1 * math.cos(ts),
        base_h + L1 * math.sin(ts)
    ])

    p2 = p1 + np.array([
        math.cos(tb) * L2 * math.cos(ts + te),
        math.sin(tb) * L2 * math.cos(ts + te),
        L2 * math.sin(ts + te)
    ])

    return np.vstack([p0, p1, p2])

# ===================== HARDWARE MAPPING (OPTIONAL) =====================
def ik_to_servo_angles(ik_angles):
    servo = {}
    for j in ik_angles:
        val = SERVO_DIRECTIONS[j] * ik_angles[j] + SERVO_OFFSETS[j]
        lo, hi = SERVO_LIMITS[j]
        servo[j] = clamp(val, lo, hi)
    return servo

# ===================== GUI =====================
app = QApplication(sys.argv)
win = QWidget()
layout = QVBoxLayout(win)
win.setWindowTitle("3D IK Arm Simulator")

view = gl.GLViewWidget()
view.setCameraPosition(distance=35, elevation=25)
layout.addWidget(view, stretch=5)

grid = gl.GLGridItem()
grid.scale(2, 2, 1)
view.addItem(grid)

arm_line = gl.GLLinePlotItem(width=4)
view.addItem(arm_line)

target_dot = gl.GLScatterPlotItem(size=10, color=(0, 1, 0, 1))
view.addItem(target_dot)

def make_slider(name, lo, hi, init):
    label = QLabel(f"{name}: {init}")
    slider = QSlider(Qt.Horizontal)
    slider.setRange(lo, hi)
    slider.setValue(init)
    layout.addWidget(label)
    layout.addWidget(slider)
    return label, slider

lbl_x, sx = make_slider("X", -20, 20, 12)
lbl_y, sy = make_slider("Y", -20, 20, 0)
lbl_z, sz = make_slider("Z", 0, 25, 10)

# ===== Option B: Angle display labels (ADDED) =====
lbl_base = QLabel("Base: 0.0°")
lbl_shoulder = QLabel("Shoulder: 0.0°")
lbl_elbow = QLabel("Elbow: 0.0°")

layout.addWidget(lbl_base)
layout.addWidget(lbl_shoulder)
layout.addWidget(lbl_elbow)

# ===================== UPDATE LOOP =====================
def update():
    global PREV_IK

    x, y, z = sx.value(), sy.value(), sz.value()
    lbl_x.setText(f"X: {x}")
    lbl_y.setText(f"Y: {y}")
    lbl_z.setText(f"Z: {z}")

    target_dot.setData(pos=np.array([[x, y, z]]))

    status = classify_reachability(x, y, z, ARM)

    ik = solve_ik_3d(x, y, z, ARM, PREV_IK)
    if ik is None:
        arm_line.setData(color=(1, 0, 0, 1))
        return

    # store previous solution for continuity
    PREV_IK = ik

    # ===== Option B: update angle labels (ADDED) =====
    lbl_base.setText(f"Base: {ik['base']:.1f}°")
    lbl_shoulder.setText(f"Shoulder: {ik['shoulder']:.1f}°")
    lbl_elbow.setText(f"Elbow: {ik['elbow']:.1f}°")

    pts = forward_kinematics(ik, ARM)

    if status == "safe":
        color = (0, 1, 0, 1)
    elif status == "near_singularity":
        color = (1, 1, 0, 1)
    else:
        color = (1, 0, 0, 1)

    arm_line.setData(pos=pts, color=color)


timer = QTimer()
timer.timeout.connect(update)
timer.start(30)

win.show()
sys.exit(app.exec_())
