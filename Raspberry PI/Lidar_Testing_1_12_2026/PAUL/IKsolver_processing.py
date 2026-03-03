import time
import math
import threading
import numpy as np

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

def _compute_pwms(x, y, z, pitch, grip=1.0, elbow_up=True, chassis_pos=(0.0, 0.0)):
    cx, cy     = chassis_pos
    arm_mount  = np.array([cx, cy, CHASSIS_LZ], dtype=float)
    tx, ty, tz = x - arm_mount[0], y - arm_mount[1], z - arm_mount[2]
    yaw0       = math.atan2(ty, tx)
    rxy        = (L3 + CLAW_CENTER_OFFSET) * math.cos(pitch)
    rz         = (L3 + CLAW_CENTER_OFFSET) * math.sin(pitch)
    sol        = _solve_ik_wrist_base(tx - rxy*math.cos(yaw0),
                                       ty - rxy*math.sin(yaw0),
                                       tz - rz, elbow_up)
    if sol is None:
        return None
    yaw, shoulder, elbow = sol
    wrist = pitch - (shoulder + elbow)
    if not _within_limits(yaw, shoulder, elbow, wrist):
        return None
    pwms = []
    for name, val in zip(["yaw","shoulder","elbow","wrist"], [yaw, shoulder, elbow, wrist]):
        for cfg_name, mid, ppr in _PWM_CONFIGS:
            if cfg_name == name:
                pwms.append(int(round(mid + ppr * val)))
    pwms.append(int(round(CLAW_CLOSE_PWM + grip * (CLAW_OPEN_PWM - CLAW_CLOSE_PWM))))
    return pwms


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
        self._pwms   = None
        self._lock   = threading.Lock()
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            with self._lock:
                t = dict(self._target)
            pwms = _compute_pwms(t["x"], t["y"], t["z"], t["pitch"],
                                  t["grip"], t["elbow_up"], t["chassis_pos"])
            if pwms is not None:
                with self._lock:
                    self._pwms = pwms
                if self.shared is not None:
                    self.shared.set_data("ik_pwms", pwms)
            time.sleep(self.poll)

    def set_target(self, x, y, z, pitch, grip=1.0, elbow_up=True, chassis_pos=(0.0, 0.0)):
        with self._lock:
            self._target = {"x": x, "y": y, "z": z, "pitch": pitch,
                            "grip": grip, "elbow_up": elbow_up,
                            "chassis_pos": chassis_pos}

    def get_pwms(self):
        with self._lock:
            return list(self._pwms) if self._pwms is not None else None

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)


def create_and_run(shared_data, poll=0.05):
    return IKProcessor(shared_data, poll=poll)
    


if __name__ == "__main__":
    print("Run this via run_all.py so it gets a shared_data instance from testingUART.")
