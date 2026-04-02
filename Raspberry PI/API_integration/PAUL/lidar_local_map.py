# lidar_local_map.py
import time
import threading
import numpy as np
from obstacle_grid_processing import fused_pose

class LidarLocalMap:
    def __init__(self, lidar_proc, poll=0.02):
        self.lidar = lidar_proc
        self.poll = poll

        self.points_world = []   # [(x,y), ...]

        self._running = True
        self._thread = threading.Thread(target=self.loop, daemon=True)
        self._thread.start()

    def loop(self):
        while self._running:
            scan = self.lidar.get_latest_scan()
            if scan is None:
                time.sleep(self.poll)
                continue

            self.project_scan(scan)
            time.sleep(self.poll)

    def project_scan(self, scan):
        x = fused_pose['x']
        y = fused_pose['y']
        theta = fused_pose['theta']
        #print(f"[LidarLocalMap] Robot pose: x={x:.2f} m, y={y:.2f} m, theta={np.degrees(theta):.1f} deg", flush=True)

        angles = scan.angles_rad
        angles = -angles
        ranges = scan.ranges_m 

        xs = x + ranges * np.cos(angles)
        ys = y + ranges * np.sin(angles)

        self.points_world = list(zip(xs, ys))
