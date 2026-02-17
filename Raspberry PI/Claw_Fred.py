import math

# -------------------------------
# Arm geometry
# -------------------------------
ARM = {
    "L1": 10,
    "L2": 10,
    "base_height": 6,
    "claw_offset": 6
}

# -------------------------------
# Servo limits (degrees)
# -------------------------------
SERVO_LIMITS = {
    "base":     (0, 180),
    "shoulder": (0, 180),
    "elbow":    (0, 180),
    "wrist":    (0, 180),
}

# -------------------------------
# Servo offsets (neutral = 90°)
# -------------------------------
SERVO_OFFSETS = {
    "base": 90,
    "shoulder": 90,
    "elbow": 90,
    "wrist": 90,
}

# -------------------------------
# Utility
# -------------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# -------------------------------
# IK solver (PURE math)
# -------------------------------
def solve_ik_3d(x, y, z, arm, elbow_up=False):
    L1 = arm["L1"]
    L2 = arm["L2"]
    base_h = arm["base_height"]
    claw = arm["claw_offset"]

    # --- Base yaw ---
    theta_base = math.atan2(y, x)

    # --- Collapse into 2D ---
    r = math.sqrt(x*x + y*y)
    r_eff = r - claw
    z_eff = z - base_h

    dist2 = r_eff*r_eff + z_eff*z_eff
    dist = math.sqrt(dist2)

    # Reachability check
    if dist > (L1 + L2) or dist < abs(L1 - L2):
        return None

    # --- Elbow ---
    D = (dist2 - L1*L1 - L2*L2) / (2*L1*L2)
    D = clamp(D, -1.0, 1.0)

    theta_elbow = math.acos(D)
    if elbow_up:
        theta_elbow = -theta_elbow

    # --- Shoulder ---
    phi = math.atan2(z_eff, r_eff)
    psi = math.atan2(
        L2 * math.sin(theta_elbow),
        L1 + L2 * math.cos(theta_elbow)
    )

    theta_shoulder = phi - psi

    # --- Wrist keeps claw level ---
    theta_wrist = -(theta_shoulder + theta_elbow)

    return {
        "base": math.degrees(theta_base),
        "shoulder": math.degrees(theta_shoulder),
        "elbow": math.degrees(theta_elbow),
        "wrist": math.degrees(theta_wrist)
    }

# -------------------------------
# IK → Servo conversion
# -------------------------------
def ik_to_servo_angles(ik_angles, offsets, limits):
    servo_angles = {}

    for joint, ik_angle in ik_angles.items():
        servo_angle = ik_angle + offsets[joint]
        servo_angle = clamp(
            servo_angle,
            limits[joint][0],
            limits[joint][1]
        )
        servo_angles[joint] = servo_angle

    return servo_angles

# -------------------------------
# Test with a simulated target
# -------------------------------
target = {"x": 5, "y": 4, "z": 5}

ik_angles = solve_ik_3d(
    target["x"],
    target["y"],
    target["z"],
    ARM
)

if ik_angles is None:
    print("Target unreachable (geometry)")
else:
    servo_angles = ik_to_servo_angles(
        ik_angles,
        SERVO_OFFSETS,
        SERVO_LIMITS
    )

    print("IK angles (math space):")
    for k, v in ik_angles.items():
        print(f"  {k}: {v:.1f}°")

    print("\nServo angles (hardware space):")
    for k, v in servo_angles.items():
        print(f"  {k}: {v:.1f}°")

