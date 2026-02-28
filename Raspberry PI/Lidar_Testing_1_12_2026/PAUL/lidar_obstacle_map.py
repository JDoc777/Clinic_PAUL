# lidar_obstacle_map.py
import time
import threading
import numpy as np
from obstacle_grid_processing import fused_pose


class LidarObstacleMap:
    """
    LiDAR log-odds update using RAW scan data.
    Does NOT depend on lidar_local_map visualization.
    """

    def __init__(self, lidar_processor, grid, poll=0.05):
        self.lidar = lidar_processor      # <-- pass lidar_map
        self.grid_obj = grid              # ObstacleGrid instance
        self.poll = poll

        self._running = True
        self._thread = threading.Thread(target=self.loop, daemon=True)
        self._thread.start()

    def loop(self):
        while self._running:
            self.update_from_lidar()
            time.sleep(self.poll)

    def update_from_lidar(self):

        scan = self.lidar.get_latest_scan()
        if scan is None:
            return

        x_robot = fused_pose['x']
        y_robot = fused_pose['y']
        theta   = fused_pose['theta']

        angles = scan.angles_rad
        ranges = scan.ranges_m 

        for a, r in zip(angles, ranges):

            

            xr = r*np.cos(a)
            yr = r*np.sin(a)

            c = np.cos(theta)
            s = np.sin(theta)
        

            x_hit = x_robot + c * xr - s * yr
            y_hit = y_robot + s * xr + c * yr

            #print(f"angles={angles}")
        

            

            self.log_odometry(x_hit, y_hit)

    # ------------------------------------------------
    # Log-odds update (same structure as sonar)
    # ------------------------------------------------
    def log_odometry(self, X_hit_world, Y_hit_world):

        l_occ  = +2.0
        l_free = -3.0

        X_robot = fused_pose['x']
        Y_robot = fused_pose['y']

        start_ix, start_iy = self.grid_obj.world_to_grid(X_robot, Y_robot)
        end_ix, end_iy     = self.grid_obj.world_to_grid(X_hit_world, Y_hit_world)

        cells = self.grid_obj.bresenham_line(
            (start_ix, start_iy),
            (end_ix, end_iy)
        )

        height, width = self.grid_obj.grid.shape

        # Free cells
        for (ix, iy) in cells[:-1]:
            if 0 <= ix < width and 0 <= iy < height:
                self.grid_obj.grid[iy, ix] += l_free

        # Occupied cell
        ix, iy = cells[-1]
        if 0 <= ix < width and 0 <= iy < height:
            self.grid_obj.grid[iy, ix] += l_occ

        # Clamp
        self.grid_obj.grid = np.clip(self.grid_obj.grid, -10, 10)
