import math
import time
import threading
import Payload
import testingUART


# =========================
# SECTION 1: tiny helpers
# =========================

def clamp(value, low, high):
    """Keep a number between low and high."""
    return max(low, min(high, value))


# =========================
# SECTION 2: base yaw from (x,y)
# =========================

def base_yaw_xy(x, y):
    """
    Base rotation angle (degrees) to face point (x,y).
    """
    return math.degrees(math.atan2(y, x))


# =========================
# SECTION 3: convert (x,y,z) -> (r,z)
# =========================

def rz_from_xyz(x, y, z):
    """
    r = horizontal distance to target (sqrt(x^2 + y^2))
    z = vertical distance (unchanged)
    """
    r = math.hypot(x, y)
    return r, z


# =========================
# SECTION 4: planar 2-link IK for shoulder+elbow
# =========================

def shoulder_elbow_deg_from_rz(r, z, L1, L2, elbow_up=False):
    """
    Solve planar 2-link IK for a target (r, z).

    r: horizontal distance from shoulder pivot
    z: vertical distance from shoulder pivot
    L1: link1 length (shoulder->elbow)
    L2: link2 length (elbow->wrist)

    Returns (shoulder_deg, elbow_deg)
    """
    # Law of cosines
    D = (r*r + z*z - L1*L1 - L2*L2) / (2.0 * L1 * L2)

    # Unreachable check
    if D < -1.0 or D > 1.0:
        raise ValueError(f"Target unreachable (D={D:.3f}). Check lengths or target.")

    # Two possible elbow solutions
    s = math.sqrt(max(0.0, 1.0 - D*D))
    if elbow_up:
        s = -s

    elbow_rad = math.atan2(s, D)

    # Shoulder angle
    shoulder_rad = math.atan2(z, r) - math.atan2(
        L2 * math.sin(elbow_rad),
        L1 + L2 * math.cos(elbow_rad)
    )

    return math.degrees(shoulder_rad), math.degrees(elbow_rad)


# =========================
# SECTION 5: full IK solve (base + shoulder + elbow + wrist)
# NOTE: this version does NOT yet include a claw offset (L3).
# =========================

def ik_solve_xyz_with_claw_offset(x, y, z, L1, L2, L3, phi_deg=0.0, elbow_up=False):
    """
    IK solve that targets the *claw grasp point* at (x,y,z).

    L3 = distance from wrist joint to the grasp-center point.
    phi_deg = desired claw pitch (0 = level).
    """
    # 1) base yaw
    base_deg = base_yaw_xy(x, y)

    # 2) convert xyz -> (r,z)
    r, z = rz_from_xyz(x, y, z)

    # 3) subtract the claw offset from the target (projected into r,z plane)
    phi = math.radians(phi_deg)
    r_wrist = r - L3 * math.cos(phi)
    z_wrist = z - L3 * math.sin(phi)

    # 4) solve shoulder & elbow to reach the wrist point
    shoulder_deg, elbow_deg = shoulder_elbow_deg_from_rz(r_wrist, z_wrist, L1, L2, elbow_up=elbow_up)

    # 5) wrist pitch to maintain desired end-effector pitch
    wrist_deg = phi_deg - shoulder_deg - elbow_deg

    return base_deg, shoulder_deg, elbow_deg, wrist_deg


# =========================
# TESTS (only run if you execute this file directly)
# =========================

if __name__ == "__main__":
    # --- clamp test (optional) ---
    print(clamp(200, 0, 180))  # should print 180

    # --- base yaw tests ---
    print(base_yaw_xy(1, 0))    # 0
    print(base_yaw_xy(0, 1))    # 90
    print(base_yaw_xy(1, 1))    # 45
    print(base_yaw_xy(-1, 0))   # 180

    # --- r,z test ---
    print(rz_from_xyz(3, 4, 10))  # (5.0, 10)

    # --- shoulder+elbow test ---
    L1 = 10
    L2 = 10
    L3 = 2
    print(shoulder_elbow_deg_from_rz(r=10, z=0, L1=L1, L2=L2, elbow_up=False))
    print(shoulder_elbow_deg_from_rz(r=10, z=0, L1=L1, L2=L2, elbow_up=True))

    # --- full IK test ---
print(ik_solve_xyz_with_claw_offset(x=10, y=0, z=0, L1=L1, L2=L2, L3=L3, phi_deg=0.0, elbow_up=False))
