import os
import sys
import time
import threading
import math
import numpy as np
from encoder_processing import robot_position
from Accel_local_map import AccelGyroProcessor, main
from sonar_obstacle_local_map import SonarObstacleLocalMap
import covariance_processing

fused_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

class ObstacleGrid:
    def __init__(self, shared_data, poll=0.5):
        self.shared = shared_data
        self.accel_processor = AccelGyroProcessor(shared_data)
        self.sonar_map = SonarObstacleLocalMap.create_and_run(shared_data=shared_data, poll=poll)
        self.poll = poll
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self.loop, daemon=True)
        self._last_time = time.time()
        self._thread.start()
        self.theta_fused = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.delta_s = 0.0
        self.cell_size = 0.1  # meters
        self.Q = np.diag([0.002, 0.002, 0.001])
        self.P = np.eye(3) * 0.01
        self.slam = None
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None
        self.setup_grid(width_m=4.0, height_m=4.0, cell_size=self.cell_size)

    def loop(self):
        

        target_dt = 0.1  # 10 Hz


        self._last_time = time.time()
        while self._running.is_set():
            start_time = time.time()
            now = time.time()
            self.dt = now - self._last_time
            self._last_time = now
            if self.dt <= 0 or self.dt > 0.1:
                self.dt = 0.01
            elapsed = time.time() - start_time
            time.sleep(max(0, target_dt - elapsed))

            # Fused position update
            self.slam = covariance_processing.SLAMSystem(self.Q, target_dt)
            self.fused_pos()
            self.odometry()
            self.sonar_map.fused_sonar_angle_processing(fused_pose)
            self.sonar_map.fused_sonar_axis_processing(fused_pose)
            self.sonar_map.fused_sonar_pos_processing()
            self.sonar_map.print_combined_data()
            self.sonar_map.fused_sonar_obstacle_axis_processing(fused_pose)
            for (x_hit, y_hit) in [
                (self.sonar_map.F_obstacle_x_fused, self.sonar_map.F_obstacle_y_fused),
                (self.sonar_map.L_obstacle_x_fused, self.sonar_map.L_obstacle_y_fused),
                (self.sonar_map.R_obstacle_x_fused, self.sonar_map.R_obstacle_y_fused),
                (self.sonar_map.B_obstacle_x_fused, self.sonar_map.B_obstacle_y_fused)
            ]:
                self.log_odometry(x_hit, y_hit)
                
            v, w = self.compute_v_w(fused_pose['x'], fused_pose['y'], fused_pose['theta'], self.accel_processor.theta, now=time.time())
            print(f"Computed velocities: v={v:.4f} m/s, w={math.degrees(w):.4f} degrees/s", flush=True)

            (fused_pose['x'], fused_pose['y'], fused_pose['theta']), P, conf = self.slam.update(v, w, fused_pose['x'], fused_pose['y'], fused_pose['theta'],
                                                              [self.sonar_map.F_raw,
                                                               self.sonar_map.L_raw,
                                                               self.sonar_map.R_raw,
                                                               self.sonar_map.B_raw],
                                                              [self.sonar_map.F_sonar_angle_fused,
                                                               self.sonar_map.L_sonar_angle_fused,
                                                               self.sonar_map.R_sonar_angle_fused,
                                                               self.sonar_map.B_sonar_angle_fused],
                                                              self.grid,
                                                              (self.x_origin, self.y_origin),
                                                              self.cell_size,
                                                              self.P)
            
            print(f"SLAM Corrected Pose: x={fused_pose['x']:.4f}, y={fused_pose['y']:.4f}, theta={math.degrees(fused_pose['theta']):.4f} degrees, confidence={conf:.4f}, P=\n{P}", flush=True)
            self.cov_log_odometry(x_hit, y_hit, conf)


    def fused_pos(self):
        a = 0.98
        # for testing purposes, we are removing the alpha when not testing add this back (1 - a) * self.accel_processor.theta
        fused_pose['theta'] = a * (robot_position['angle']) + self.accel_processor.theta
        fused_pose['theta'] = fused_pose['theta'] % (2 * math.pi)  # Clamp between 0 and 2π
        print(f"Fused angle (degrees): {math.degrees(fused_pose['theta']):.4f}")
        return fused_pose['theta']

    def odometry(self):
        fused_pose['x'] = robot_position['x']
        fused_pose['y'] = robot_position['y']
        print(f"Fused position: x={fused_pose['x']:.4f} m, y={fused_pose['y']:.4f} m")
        return fused_pose['x'], fused_pose['y']
    
    def setup_grid(self, width_m, height_m, cell_size):
        width = int(width_m / cell_size)
        height = int(height_m / cell_size)
        self.grid = np.zeros((height, width), dtype=int)
        # Center the robot at the middle of the grid at start
        self.x_origin = - (width_m / 2.0)
        self.y_origin = - (height_m / 2.0)
        print(f"Grid shape: {self.grid.shape}")

    def world_to_grid(self, x_world, y_world):
        ix = int((x_world - self.x_origin) / self.cell_size)
        iy = int((y_world - self.y_origin) / self.cell_size)
        return ix, iy

    @staticmethod   
    def bresenham_line(start, end):
        """
        Bresenham's Line Algorithm
        Returns a list of (x, y) grid cells between start and end (inclusive)
        start, end = (x0, y0), (x1, y1)
        """
        x0, y0 = start
        x1, y1 = end

        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells
    
    def log_odometry(self, X_hit_world, Y_hit_world):
        l_occ = +2.0   # log-odds increment for occupied
        l_free = -1.0  # log-odds increment for free

        X_new = fused_pose['x']
        Y_new = fused_pose['y']

        start_ix, start_iy = self.world_to_grid(X_new, Y_new)
        end_ix, end_iy     = self.world_to_grid(X_hit_world, Y_hit_world)

        cells = self.bresenham_line((start_ix, start_iy), (end_ix, end_iy))

        height, width = self.grid.shape

        # mark free cells along the ray
        for (ix, iy) in cells[:-1]:
            if 0 <= ix < width and 0 <= iy < height:
                self.grid[iy, ix] += l_free

        # mark the last cell (hit point) as occupied
        ix, iy = cells[-1]
        if 0 <= ix < width and 0 <= iy < height:
            self.grid[iy, ix] += l_occ

        # clamp to range
        self.grid = np.clip(self.grid, -10, 10)

        prob_map = 1 / (1 + np.exp(-self.grid))  # convert when needed only use for visualization or interpretation
        return prob_map
    
    def cov_log_odometry(self, X_hit_world, Y_hit_world, conf_interval):
        l_occ = +2.0   # log-odds increment for occupied
        l_free = -1.0  # log-odds increment for free

        X_new = fused_pose['x']
        Y_new = fused_pose['y']

        start_ix, start_iy = self.world_to_grid(X_new, Y_new)
        end_ix, end_iy     = self.world_to_grid(X_hit_world, Y_hit_world)

        cells = self.bresenham_line((start_ix, start_iy), (end_ix, end_iy))

        height, width = self.grid.shape

        # mark free cells along the ray
        for (ix, iy) in cells[:-1]:
            if 0 <= ix < width and 0 <= iy < height:
                self.grid[iy, ix] += l_free

        # mark the last cell (hit point) as occupied
        ix, iy = cells[-1]
        if 0 <= ix < width and 0 <= iy < height:
            self.grid[iy, ix] += l_occ * conf_interval

        # clamp to range
        self.grid = np.clip(self.grid, -10, 10)

        prob_map = 1 / (1 + np.exp(-self.grid))  # convert when needed only use for visualization or interpretation
        return prob_map
    
    def compute_v_w(self, x, y, theta, gyro_z, now, default_dt=0.05):
        # first call – nothing to diff against
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_theta = theta
            self.prev_time = now
            return 0.0, gyro_z

        dt = now - self.prev_time if self.prev_time is not None else default_dt
        if dt <= 0:
            dt = default_dt

        dx = x - self.prev_x
        dy = y - self.prev_y

        vx_world = dx / dt
        vy_world = dy / dt

        v_fused = vx_world * np.cos(theta) + vy_world * np.sin(theta)

        dtheta = theta - self.prev_theta
        dtheta = (dtheta + np.pi) % (2*np.pi) - np.pi
        w_from_pose = dtheta / dt

        # choose what you want here:
        w_fused = gyro_z  # or blend gyro + pose
        # alpha = 0.9
        # w_fused = alpha*gyro_z + (1-alpha)*w_from_pose

        # update history
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        self.prev_time = now

        return v_fused, w_fused

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

def create_and_run(shared_data, poll=0.5):
    return ObstacleGrid(shared_data, poll=poll)


def main(poll_interval=0.5):  # 100 Hz default
    # Start the shared UART reader (reader-only, no writer)
    import testingUART
    shared_data, running_event, ser = testingUART.start_reader(start_writer=False, do_handshake=False)
    print("ObstacleGrid: started reader (reader-only). Press Ctrl-C to exit.")
    grid = ObstacleGrid(shared_data, poll=poll_interval)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        running_event.clear()
        time.sleep(0.2)
        try:
            testingUART.close_serial()
        except Exception:
            pass
        print("Stopped.")

def run():
    """Entry point for run_all: runs main() with default settings."""
    main()

if __name__ == "__main__":
    run()



