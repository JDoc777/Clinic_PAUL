import numpy as np
import math
from encoder_processing import robot_position
from obstacle_grid_processing import fused_pose


def ekf_predict(x_prev, P_prev, u, dt, Q):
    # x = [x, y, theta], u = [v, w]
    x, y, th = x_prev
    v, w = u

    # Motion model (unicycle)
    x_pred = np.array([
        x + v * dt * math.cos(th),
        y + v * dt * math.sin(th),
        th + w * dt
    ])
    # normalize angle to [-pi, pi]
    x_pred[2] = (x_pred[2] + math.pi) % (2*math.pi) - math.pi

    # Jacobian wrt state
    F = np.array([
        [1.0, 0.0, -v * dt * math.sin(th)],
        [0.0, 1.0,  v * dt * math.cos(th)],
        [0.0, 0.0,  1.0]
    ])

    P_pred = F @ P_prev @ F.T + Q
    return x_pred, P_pred

def _estimate_controls(self, dt):
    """
    Estimate linear v and angular w using your existing sources.
    v from encoder odometry (robot_position), w from gyro+fusion.
    Falls back to finite-difference if gyro is not available.
    """
    # positions from encoders (already in meters in your code)
    x = robot_position['x']
    y = robot_position['y']

    # fused theta (your alpha blend)
    th_fused = self.fused_pos()  # updates fused_pose['theta'] and returns it

    # linear speed from encoder displacement
    dx = x - self._last_pose_for_u['x']
    dy = y - self._last_pose_for_u['y']
    ds = math.hypot(dx, dy)
    v = (ds / dt) if dt > 1e-6 else 0.0

    # angular rate: prefer gyro if AccelGyroProcessor exposes it, else finite-diff
    if hasattr(self.accel_processor, 'omega_z'):
        w = float(self.accel_processor.omega_z)   # rad/s from IMU if available
    else:
        dth = (th_fused - self._last_pose_for_u['theta'])
        # wrap to [-pi, pi]
        dth = (dth + math.pi) % (2*math.pi) - math.pi
        w = (dth / dt) if dt > 1e-6 else 0.0

    # stash for next round
    self._last_pose_for_u['x'] = x
    self._last_pose_for_u['y'] = y
    self._last_pose_for_u['theta'] = th_fused

    return np.array([v, w]), th_fused

