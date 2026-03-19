# sec3_control_real.py

import time
import threading
import numpy as np

from obstacle_grid_processing import fused_pose
import Payload

from LocomotionSIM2 import (
    angle_wrap,
    mecanum_ack,
    mecanum_tip,
    get_forward_heading,
    BOT_SPEED_MPS,
    BOT_ACCEL,
    BOT_DECEL,
    TIP_STOP_TOL,
    TIP_SLOW_RADIUS_M,
    TIP_STOP_RADIUS_M,
    DENSIFY_STEP_M,
    HEADING_LOOKAHEAD_PTS,
    MAX_HEADING_JUMP_RAD,
)


class RealSEC3:
    def __init__(self, sec2_plot, poll=0.02):
        self.sec2_plot = sec2_plot
        self.poll = float(poll)
        self.running = True

        # Section 3 state
        self.mode = "ACK"
        self.turn_target_theta = None

        self.prev_desired_theta = 0.0

        self.vx_current = 0.0
        self.SCALE = 80  # Scale factor for converting m/s to motor command units (this is a placeholder and should be tuned based on your robot's characteristics)
        self.vx_target = BOT_SPEED_MPS
        self.accel = BOT_ACCEL
        self.decel = BOT_DECEL

        self.tip_set = set()
        self.path_index = 0

        self.render_pos = np.zeros(2, dtype=float)
        self.render_theta = 0.0
        self.render_text = "Initializing..."

        self.wheel_speeds = np.zeros(4, dtype=float)

        self._have_initialized_heading = False

    def find_closest_index(self, path: np.ndarray, pos: np.ndarray) -> int:
        d = np.linalg.norm(path - pos, axis=1)
        return int(np.argmin(d))

    def get_remaining_to_next_tip_m(self, path_index: int) -> float:
        next_tips = [t for t in self.tip_set if t >= path_index]
        if not next_tips:
            return 999999.0
        return (min(next_tips) - path_index) * DENSIFY_STEP_M

    def stop_motion(self, pos: np.ndarray, theta: float, text: str = "DONE") -> None:
        self.wheel_speeds = np.zeros(4, dtype=float)
        self.render_pos = pos.copy()
        self.render_theta = float(theta)
        self.render_text = text

    def step(self):
        data = self.sec2_plot.latest_data
        if data is None:
            return

        modified_path = np.asarray(data["modified_path"], dtype=float)
        tip_indices = list(data["tip_indices"])

        if len(modified_path) < 2:
            return

        self.tip_set = set(tip_indices)

        # REAL pose only. Do not simulate pose here.
        pos = np.array([fused_pose["x"], fused_pose["y"]], dtype=float)
        theta = float(fused_pose["theta"])

        self.render_pos = pos.copy()
        self.render_theta = theta

        self.path_index = self.find_closest_index(modified_path, pos)

        if not self._have_initialized_heading:
            self.prev_desired_theta = theta
            self._have_initialized_heading = True

        # stop at end
        if self.path_index >= len(modified_path) - 1:
            self.stop_motion(pos, theta, "DONE")
            Payload.set_motors(self.sec2_plot.grid.shared, 0.0, 0.0, 0.0, 0.0)
            return

        # ===============================
        # ACK MODE
        # ===============================
        if self.mode == "ACK":
            desired_theta = get_forward_heading(
                modified_path,
                self.path_index,
                default_theta=theta,
                look_pts=HEADING_LOOKAHEAD_PTS
            )

            jump = angle_wrap(desired_theta - self.prev_desired_theta)
            if abs(jump) > MAX_HEADING_JUMP_RAD:
                desired_theta = self.prev_desired_theta

            self.prev_desired_theta = float(desired_theta)

            error = angle_wrap(desired_theta - theta)

            K_OMEGA = 0.5        # start here
            MAX_OMEGA = 1.0      # rad/s clamp

            omega_cmd = np.clip(K_OMEGA * error, -MAX_OMEGA, MAX_OMEGA)

            #OMEGA CLAMP MAY CHANGE BECAUSE BOT SPEED CHANGED. TEST AND TUNE.

            remaining = self.get_remaining_to_next_tip_m(self.path_index)

            if remaining < TIP_STOP_RADIUS_M:
                self.vx_target = 0.0
                accel_used = self.decel
            elif remaining < TIP_SLOW_RADIUS_M:
                self.vx_target = BOT_SPEED_MPS * 0.5
                accel_used = self.decel
            else:
                self.vx_target = BOT_SPEED_MPS
                accel_used = self.accel

            self.vx_current += accel_used * (self.vx_target - self.vx_current)
            vx_cmd = self.vx_current

            wheels = mecanum_ack(vx_cmd, omega_cmd)
            self.wheel_speeds = np.asarray(wheels, dtype=float)

            theta_deg = np.degrees(theta) % 360.0
            self.render_text = (
                f"ACK MODE\n"
                f"w1={wheels[0]:.2f}\n"
                f"w2={wheels[1]:.2f}\n"
                f"w3={wheels[2]:.2f}\n"
                f"w4={wheels[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )

            Payload.set_motors(
                self.sec2_plot.grid.shared, 
                self.wheel_speeds[0] * self.SCALE, 
                self.wheel_speeds[1] * self.SCALE, 
                self.wheel_speeds[2] * self.SCALE, 
                self.wheel_speeds[3] * self.SCALE)

            # transition into TIP when robot reaches a TIP index
            if self.path_index in self.tip_set:
                self.turn_target_theta = get_forward_heading(
                    modified_path,
                    self.path_index,
                    default_theta=theta,
                    look_pts=HEADING_LOOKAHEAD_PTS
                )
                self.mode = "TIP"
                self.vx_current = 0.0
                return

        # ===============================
        # TIP MODE
        # ===============================
        elif self.mode == "TIP":
            if self.turn_target_theta is None:
                self.mode = "ACK"
                self.stop_motion(pos, theta, "TIP target missing")
                Payload.set_motors(self.sec2_plot.grid.shared, 0.0, 0.0, 0.0, 0.0)  
                return

            dtheta = angle_wrap(self.turn_target_theta - theta)

            if abs(dtheta) < TIP_STOP_TOL:
                self.mode = "ACK"
                self.wheel_speeds = np.zeros(4, dtype=float)
                self.render_text = (
                    f"TIP ROTATION COMPLETE\n"
                    f"theta={np.degrees(theta) % 360.0:.1f}°\n"
                    f"idx={self.path_index}"
                )
                return

            wheels_tip = mecanum_tip(dtheta)
            self.wheel_speeds = np.asarray(wheels_tip, dtype=float)


            theta_deg = np.degrees(theta) % 360.0
            
            self.render_text = (
                f"TIP ROTATION\n"
                f"w1={wheels_tip[0]:.2f}\n"
                f"w2={wheels_tip[1]:.2f}\n"
                f"w3={wheels_tip[2]:.2f}\n"
                f"w4={wheels_tip[3]:.2f}\n"
                f"theta={theta_deg:.1f}°\n"
                f"idx={self.path_index}"
            )
            Payload.set_motors(
                self.sec2_plot.grid.shared, 
                self.wheel_speeds[0] * self.SCALE, 
                self.wheel_speeds[1] * self.SCALE, 
                self.wheel_speeds[2] * self.SCALE, 
                self.wheel_speeds[3] * self.SCALE)

        else:
            self.stop_motion(pos, theta, f"Unknown mode: {self.mode}")

    def run(self):
        while self.running:
            try:
                self.step()
            except Exception as e:
                self.wheel_speeds = np.zeros(4, dtype=float)
                self.render_text = f"SEC3 ERROR: {e}"
            time.sleep(self.poll)


def create_and_run(sec2_plot, poll=0.02):
    sec3 = RealSEC3(sec2_plot, poll=poll)
    thread = threading.Thread(target=sec3.run, daemon=True)
    thread.start()
    return sec3