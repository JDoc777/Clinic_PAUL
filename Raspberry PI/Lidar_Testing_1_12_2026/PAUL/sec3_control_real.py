# sec3_control_real.py

import time
import threading
import numpy as np
import csv
import atexit


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
        self.done = False
        self.turn_target_theta = None

        self.prev_desired_theta = 0.0

        self.vx_current = 0.0
        self.SCALE = 0  # Scale factor for converting m/s to motor command units (this is a placeholder and should be tuned based on your robot's characteristics)
        self.vx_target = BOT_SPEED_MPS * 0.3
        self.accel = BOT_ACCEL
        self.decel = BOT_DECEL

        self.tip_set = set()
        self.path_index = 0

        self.start_time = time.time()

        self.csv_file = open("debug_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # header
        self.csv_writer.writerow([
            "time",
            "vx_cmd",
            "omega_cmd",
            "w1", "w2", "w3", "w4",
            "heading_error",
            "mode"
        ])

        # ensure file closes even if crash
        atexit.register(self.csv_file.close)

        

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

    def find_projection_on_path(self, path, pos):
        min_dist = float("inf")
        best_idx = 0
        best_proj = path[0]

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            v = p2 - p1
            w = pos - p1

            t = np.dot(w, v) / (np.dot(v, v) + 1e-6)
            t = np.clip(t, 0.0, 1.0)

            proj = p1 + t * v
            dist = np.linalg.norm(pos - proj)

            if dist < min_dist:
                min_dist = dist
                best_idx = i
                best_proj = proj

        return best_idx, best_proj, min_dist

    def step(self):

        # ===== HARD STOP LATCH =====
        if self.done:
            Payload.set_motors(self.sec2_plot.grid.shared, 0.0, 0.0, 0.0, 0.0)
            return

        # default values (prevents crashes)
        vx_cmd = 0.0
        omega_cmd = 0.0

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
        raw_theta = float(fused_pose["theta"])

        self.theta_filtered = getattr(self, "theta_filtered", raw_theta)

        alpha_theta = 0.9   # 0.85–0.95 range
        self.theta_filtered = alpha_theta * self.theta_filtered + (1 - alpha_theta) * raw_theta

        theta = self.theta_filtered


        goal = modified_path[-1]
        dist_to_goal = np.linalg.norm(pos - goal)


        if not self._have_initialized_heading:
            self.prev_desired_theta = theta
            self._have_initialized_heading = True

        self.render_pos = pos.copy()
        self.render_theta = theta


        # slow down near goal
        if dist_to_goal < 0.25:
            self.vx_target = BOT_SPEED_MPS * 0.2

        if dist_to_goal < 0.15:   # slightly bigger radius = safer stop
            self.done = True

            self.vx_current = 0.0
            self.vx_target = 0.0

            self.stop_motion(pos, theta, "DONE")

            Payload.set_motors(self.sec2_plot.grid.shared, 0.0, 0.0, 0.0, 0.0)
            return

        # now do path tracking
        closest_idx, proj_point, cte = self.find_projection_on_path(modified_path, pos)
        self.path_index = closest_idx


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

            # sign of cross-track error
            path_dir = modified_path[min(self.path_index + 1, len(modified_path) - 1)] - modified_path[self.path_index]
            normal = np.array([-path_dir[1], path_dir[0]])
            cte_sign = np.sign(np.dot(normal, pos - proj_point))

            # ===== FIXED OMEGA CONTROL =====
            K_OMEGA = 0.15
            K_CTE = 0.05

            omega_cmd = K_OMEGA * error

            # small bias to fight drift
            BIAS = 0.003
            omega_cmd += BIAS * np.sign(error)

            # --- SMOOTH OMEGA ---
            alpha = 0.95

            self.prev_omega = getattr(self, "prev_omega", 0.0)
            omega_cmd = alpha * self.prev_omega + (1 - alpha) * omega_cmd
            self.prev_omega = omega_cmd

            # clamp BEFORE smoothing
            omega_cmd = np.clip(omega_cmd, -0.2, 0.2)
            # ==============================




            #OMEGA CLAMP MAY CHANGE BECAUSE BOT SPEED CHANGED. TEST AND TUNE.

            remaining = self.get_remaining_to_next_tip_m(self.path_index)

            if remaining < TIP_STOP_RADIUS_M:
                self.vx_target = BOT_SPEED_MPS * 0.2   # don’t fully stop
                accel_used = self.decel
            elif remaining < TIP_SLOW_RADIUS_M:
                self.vx_target = BOT_SPEED_MPS * 0.5
                accel_used = self.decel
            else:
                self.vx_target = BOT_SPEED_MPS
                accel_used = self.accel

            self.vx_current += accel_used * (self.vx_target - self.vx_current)
            vx_cmd = self.vx_current

            # NOW apply ratio limit
            if abs(vx_cmd) > 1e-4:
                omega_cmd = np.clip(omega_cmd, -0.8 * vx_cmd, 0.8 * vx_cmd)

            

        
            wheels = mecanum_ack(vx_cmd, omega_cmd)
            self.wheel_speeds = np.asarray(wheels, dtype=float)

            # ===== ADD THIS BLOCK RIGHT HERE =====
            t = time.time() - self.start_time

            self.csv_writer.writerow([
                t,
                vx_cmd,
                omega_cmd,
                wheels[0],
                wheels[1],
                wheels[2],
                wheels[3],
                error,
                self.mode
            ])
            # ====================================

            self.csv_file.flush()



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

            print(f"vx={vx_cmd:.3f}, omega={omega_cmd:.3f}")
            print(f"wheels (raw) = {wheels}")
            print("------")

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
            dtheta *= 0.5

            if abs(dtheta) < TIP_STOP_TOL:
                self.mode = "ACK"
                self.wheel_speeds = np.zeros(4, dtype=float)
                self.path_index += 2


                self.render_text = (
                    f"TIP ROTATION COMPLETE\n"
                    f"theta={np.degrees(theta) % 360.0:.1f}°\n"
                    f"idx={self.path_index}"
                )
                return

            wheels_tip = mecanum_tip(dtheta)
            MAX_TIP = 2.0   # try 1.5–2.5
            wheels_tip = np.clip(wheels_tip, -MAX_TIP, MAX_TIP)


            self.wheel_speeds = np.asarray(wheels_tip, dtype=float)

            t = time.time() - self.start_time

            self.csv_writer.writerow([
                t,
                0.0,
                dtheta,
                self.wheel_speeds[0],
                self.wheel_speeds[1],
                self.wheel_speeds[2],
                self.wheel_speeds[3],
                dtheta,
                self.mode
            ])

            self.csv_file.flush()


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

            print(f"TIP MODE")
            print(f"wheels = {self.wheel_speeds}")


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