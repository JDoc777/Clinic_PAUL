import time
import math
import threading
import numpy as np

import Payload  # uses Payload.set_servos(shared, s0..s4)

# ============================================================
# ARM GEOMETRY
# ============================================================
BASE_HEIGHT = 20.0
L1 = 152.4
L2 = 228.6
L3 = 40.0
CLAW_CENTER_OFFSET = 20.0
CHASSIS_LZ = 120.0

JOINT_LIMITS = {
    "yaw":      (math.radians( -90), math.radians( 90)),
    "shoulder": (math.radians( -45), math.radians(135)),
    "elbow":    (math.radians(-180), math.radians(  0)),
    "wrist":    (math.radians( -90), math.radians( 90)),
}
LIMIT_BUFFER = math.radians(5)

HOME_ANGLES = [
    0.0,
    math.radians( 45),
    math.radians(-90),
    0.0,
]

# ============================================================
# SERVO PWM CALIBRATION
# (pulse_mid = PWM at home angle, pulse_per_rad = µs per radian)
# ============================================================
_PWM_CONFIGS = [
    ("yaw",      1500, 572.96),
    ("shoulder", 1500, 572.96),
    ("elbow",    1500, 572.96),
    ("wrist",    1500, 572.96),
]
CLAW_OPEN_PWM  = 2000
CLAW_CLOSE_PWM =  800

# ============================================================
# Utility
# ============================================================
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _within_limits(yaw, shoulder, elbow, wrist):
    buf = LIMIT_BUFFER
    return (
        JOINT_LIMITS["yaw"][0]      + buf <= yaw      <= JOINT_LIMITS["yaw"][1]      - buf and
        JOINT_LIMITS["shoulder"][0] + buf <= shoulder <= JOINT_LIMITS["shoulder"][1] - buf and
        JOINT_LIMITS["elbow"][0]    + buf <= elbow    <= JOINT_LIMITS["elbow"][1]    - buf and
        JOINT_LIMITS["wrist"][0]    + buf <= wrist    <= JOINT_LIMITS["wrist"][1]    - buf
    )

def _solve_ik_wrist_base(x, y, z, elbow_up=True):
    yaw   = math.atan2(y, x)
    r     = math.sqrt(x*x + y*y)
    z_eff = z - BASE_HEIGHT
    d2    = r*r + z_eff*z_eff
    d     = math.sqrt(d2)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None
    cos_e    = _clamp((d2 - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    elbow    = math.acos(cos_e)
    if elbow_up:
        elbow = -elbow
    phi      = math.atan2(z_eff, r)
    shoulder = phi - math.atan2(L2*math.sin(elbow), L1 + L2*math.cos(elbow))
    return yaw, shoulder, elbow

def _angle_to_servo_degrees(joint_name: str, angle_rad: float) -> int:
    """
    Map a joint angle (radians) into a 0..180 'servo degree' command using JOINT_LIMITS.
    Lower joint limit -> 0°, upper joint limit -> 180°.
    """
    lo, hi = JOINT_LIMITS[joint_name]
    span = hi - lo
    if abs(span) < 1e-12:
        return 90
    norm = _clamp((angle_rad - lo) / span, 0.0, 1.0)
    return int(round(norm * 180.0))


def _compute_servos(x, y, z, pitch, grip=1.0, elbow_up=True, chassis_pos=(0.0, 0.0)):
    """
    Compute *servo degree commands* (0..180) for yaw/shoulder/elbow/wrist/claw.

    Note: This returns values intended for Payload.set_servos(), which clamps to [0,180]
    and packs them as uint8 in the outgoing command payload.
    """
    cx, cy     = chassis_pos
    arm_mount  = np.array([cx, cy, CHASSIS_LZ], dtype=float)
    tx, ty, tz = x - arm_mount[0], y - arm_mount[1], z - arm_mount[2]

    yaw0       = math.atan2(ty, tx)

    # Back out wrist base position from tool center + desired pitch.
    rxy        = (L3 + CLAW_CENTER_OFFSET) * math.cos(pitch)
    rz         = (L3 + CLAW_CENTER_OFFSET) * math.sin(pitch)

    sol = _solve_ik_wrist_base(
        tx - rxy * math.cos(yaw0),
        ty - rxy * math.sin(yaw0),
        tz - rz,
        elbow_up
    )
    if sol is None:
        return None

    yaw, shoulder, elbow = sol
    wrist = pitch - (shoulder + elbow)

    if not _within_limits(yaw, shoulder, elbow, wrist):
        return None

    # Joint angles -> 0..180 servo commands
    s_yaw      = _angle_to_servo_degrees("yaw", yaw)
    s_shoulder = _angle_to_servo_degrees("shoulder", shoulder)
    s_elbow    = _angle_to_servo_degrees("elbow", elbow)
    s_wrist    = _angle_to_servo_degrees("wrist", wrist)

    # Grip convention:
    #   grip = 0.0 -> closed, grip = 1.0 -> open  (matches your previous PWM formula)
    grip = _clamp(grip, 0.0, 1.0)
    s_claw = int(round(grip * 180.0))

    return [s_yaw, s_shoulder, s_elbow, s_wrist, s_claw]


# Backwards-compat wrapper (old name). Now returns servo degrees, not microseconds.
def _compute_pwms(x, y, z, pitch, grip=1.0, elbow_up=True, chassis_pos=(0.0, 0.0)):
    return _compute_servos(x, y, z, pitch, grip, elbow_up, chassis_pos)


# ============================================================
# IKProcessor
# ============================================================
class IKProcessor:
    def __init__(self, shared_data, poll=0.05):
        self.shared  = shared_data
        self.poll    = poll
        self._target = {"x": 200.0, "y": 0.0, "z": CHASSIS_LZ + 150.0,
                        "pitch": 0.0, "grip": 1.0, "elbow_up": True,
                        "chassis_pos": (0.0, 0.0)}
        self._servos = None
        self._lock   = threading.Lock()
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            with self._lock:
                t = dict(self._target)
            servos = _compute_servos(t["x"], t["y"], t["z"], t["pitch"],
                                  t["grip"], t["elbow_up"], t["chassis_pos"])
            if servos is not None:
                with self._lock:
                    self._servos = servos
                if self.shared is not None:
                    self.shared.set_data("ik_servos", servos)
                    Payload.set_servos(self.shared, *servos)
            time.sleep(self.poll)

    def set_target(self, x, y, z, pitch, grip=1.0, elbow_up=True, chassis_pos=(0.0, 0.0)):
        with self._lock:
            self._target = {"x": x, "y": y, "z": z, "pitch": pitch,
                            "grip": grip, "elbow_up": elbow_up,
                            "chassis_pos": chassis_pos}

    def get_servos(self):
        with self._lock:
            return list(self._servos) if self._servos is not None else None

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)


def create_and_run(shared_data, poll=0.05):
    return IKProcessor(shared_data, poll=poll)
    


if __name__ == "__main__":
    print("Run this via run_all.py so it gets a shared_data instance from testingUART.")
