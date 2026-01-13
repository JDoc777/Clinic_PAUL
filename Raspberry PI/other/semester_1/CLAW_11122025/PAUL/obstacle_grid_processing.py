import os
import sys
import time
import threading
import math
import numpy as np
from encoder_processing import robot_position
from Accel_local_map import AccelGyroProcessor, main
from sonar_obstacle_local_map import SonarObstacleLocalMap


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



