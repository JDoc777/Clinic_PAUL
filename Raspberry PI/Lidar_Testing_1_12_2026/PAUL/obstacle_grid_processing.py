import os
import sys
import time
import threading
import math
from cv2 import threshold
import numpy as np
from encoder_processing import robot_position
from Accel_local_map import AccelGyroProcessor, main
from sonar_obstacle_local_map import SonarObstacleLocalMap
from pathfinding import astar, grid_to_world, add_headings, inflate_obstacles, bezier
import covariance_processing
from waypoint_follower import waypoint_step
import matplotlib.pyplot as plt  # Add this import for plotting



fused_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

current_wp_index = 0
WAYPOINT_TOL = 0.15  # 15 cm



class ObstacleGrid:
    def __init__(self, shared_data, poll=0.05):
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
        self.goal_x = 1.0
        self.goal_y = 1.0
        self.cell_size = 0.05  # meters
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_theta_enc = 0.0
        self.raw_theta_gyro = 0.0
        self._last_debug = 0  # force immediate debug print
        self._last_goal_update = 0  # New variable for goal update timer
        self.current_waypoints = []
        self.current_wp_index = 0
        self.current_path_grid = None
        self.wp_x = 0.0
        self.wp_y = 0.0
        self.wp_theta = 0.0
        self.navigation_done = False
        self.prev_goal = None
        self._last_replan_time = 0.0
        self.REPLAN_COOLDOWN = 2  # seconds (tune this)

        
        self.Q = np.diag([0.005, 0.005, math.radians(1.0)])  # Process noise covariance
        self.P = np.eye(3) * 0.01

        # SLAM dt (EKF internal timestep)
        self.dt = 0.1  # 10 Hz
        # ❗ create SLAM **once**, do NOT recreate in the loop
        self.slam = covariance_processing.SLAMSystem(self.Q, self.dt)

        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None
        self.setup_grid(width_m=10, height_m=10, cell_size=self.cell_size)
                # --- Selective decay setup ---
        self.observed_mask = np.zeros_like(self.grid, dtype=bool)
        self.DECAY = 0.9995  # tune this (0.995 slow, 0.98 faster)


    def loop(self):
        

        target_dt = 0.005  # 10 Hz


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


            self.raw_x  = robot_position['x']
            self.raw_y  = robot_position['y']
            self.raw_theta_enc = robot_position['angle']
            self.raw_theta_gyro = self.accel_processor.theta

            v, w = self.compute_v_w_raw(
                self.raw_x, self.raw_y, self.raw_theta_enc, self.raw_theta_gyro, now)
            

            # Fused position update
            #self.slam = covariance_processing.SLAMSystem(self.Q, target_dt)
            #self.fused_pos()
            
            
            (x, y, theta), self.P, conf = self.slam.update(
                v, w, self.raw_x, self.raw_y, self.raw_theta_enc,
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
                self.P
                )
            
            fused_pose['x'] = x
            fused_pose['y'] = y
            fused_pose['theta'] = theta

        
            self.sonar_map.print_combined_data()
            self.sonar_map.fused_sonar_angle_processing(fused_pose)
            self.sonar_map.fused_sonar_axis_processing(fused_pose)
            self.sonar_map.fused_sonar_pos_processing()
            self.sonar_map.fused_sonar_obstacle_axis_processing(fused_pose)

            for (x_hit, y_hit) in [
                (self.sonar_map.F_obstacle_x_fused, self.sonar_map.F_obstacle_y_fused),
                (self.sonar_map.L_obstacle_x_fused, self.sonar_map.L_obstacle_y_fused),
                (self.sonar_map.R_obstacle_x_fused, self.sonar_map.R_obstacle_y_fused),
                (self.sonar_map.B_obstacle_x_fused, self.sonar_map.B_obstacle_y_fused)
            ]:
                self.log_odometry(x_hit, y_hit, conf)

                binary_map = (self.grid > 8).astype(int)


            # Debug print every second
            if time.time() - self._last_debug >= 1.0:
                #print(f"SLAM Corrected Pose: x={fused_pose['x']:.4f}, y={fused_pose['y']:.4f}, theta={math.degrees(fused_pose['theta']):.4f} degrees, confidence={conf:.4f}, P=\n{self.P}", flush=True)
                #print(f"Computed velocities: v={v:.4f} m/s, w={math.degrees(w):.4f} degrees/s", flush=True)
                self._last_debug = time.time()  
            if self.goal_x is not None:

                current_goal = (self.goal_x, self.goal_y)

                if self.current_path_grid is None:
                    print("[REPLAN] No path exists.")
                    self.try_replan(current_goal)

                elif current_goal != self.prev_goal:
                    print("[REPLAN] Goal changed.")
                    self.try_replan(current_goal)
                    self.prev_goal = current_goal

                elif self.path_is_blocked():
                    print("[REPLAN] Path blocked.")
                    self.try_replan(current_goal)

                elif self.robot_far_from_path():
                    print("[REPLAN] Robot deviated from path.")
                    self.try_replan(current_goal)

            #print("Calling waypoint_step now...")
            if self.current_waypoints and not self.navigation_done:
                #print("Current waypoint index before step:")
                new_index = waypoint_step(
                    self.current_waypoints,
                    self.current_wp_index,
                    fused_pose
                )

                if new_index != self.current_wp_index:
                    #print(f"Waypoint index updated from {self.current_wp_index} to {new_index}")
                    self.current_wp_index = new_index

                    if self.current_wp_index >= len(self.current_waypoints):
                        #print("All waypoints reached. Navigation done.")
                        self.navigation_done = True
                

            # Only if we still have waypoints
            if not self.navigation_done:
                if self.current_wp_index < len(self.current_waypoints):
                    self.wp_x, self.wp_y, self.wp_theta = self.current_waypoints[self.current_wp_index]
                    #print(f"wp_x: {self.wp_x}, wp_y: {self.wp_y}, wp_theta: {self.wp_theta}")
                else:
                    self.wp_x, self.wp_y = self.goal_x, self.goal_y   # fallback to final goal
            else:
                self.wp_x, self.wp_y, self.wp_theta = fused_pose['x'], fused_pose['y'], fused_pose['theta']  # stay put
        # ---------------------------
        # Selective decay (once per loop)
        # ---------------------------
            self.grid[~self.observed_mask] *= self.DECAY

            # Reset mask for next cycle
            self.observed_mask.fill(False)



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
        self.grid = np.zeros((height, width), dtype=np.float32)
        # Center the robot at the middle of the grid at start
        self.x_origin = - (width_m / 2.0)
        self.y_origin = - (height_m / 2.0)
        #print(f"Grid shape: {self.grid.shape}")

    def world_to_grid(self, x_world, y_world):
        ix = self.grid.shape[0] - 1 - int((x_world - self.x_origin) / self.cell_size)
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
    
    def log_odometry(self, X_hit_world, Y_hit_world, conf_interval):
        l_occ = +2.0   # log-odds increment for occupied
        l_free = -3.0  # log-odds increment for free

        X_new = fused_pose['x']
        Y_new = fused_pose['y']

        start_ix, start_iy = self.world_to_grid(X_new, Y_new)
        end_ix, end_iy     = self.world_to_grid(X_hit_world, Y_hit_world)

        cells = self.bresenham_line((start_ix, start_iy), (end_ix, end_iy))

        height, width = self.grid.shape

        for (ix, iy) in cells[:-1]:
            if 0 <= ix < width and 0 <= iy < height:
                self.grid[iy, ix] += l_free
                self.observed_mask[iy, ix] = True


        # mark the last cell (hit point) as occupied
        ix, iy = cells[-1]
        if 0 <= ix < width and 0 <= iy < height:
            self.grid[iy, ix] += l_occ * conf_interval
            self.observed_mask[iy, ix] = True

        # clamp to range
        self.grid = np.clip(self.grid, -10, 10)

        prob_map = 1 / (1 + np.exp(-self.grid))  # convert when needed only use for visualization or interpretation
        
        return prob_map
    
    def cov_log_odometry(self, X_hit_world, Y_hit_world, conf_interval):
        l_occ = +3.0   # log-odds increment for occupied
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
    
    def compute_v_w_raw(self, x, y, theta, gyro_z, now, default_dt=0.1):
        #print(f"Computing v and w with pose x={x:.4f}, y={y:.4f}, theta={math.degrees(theta):.4f} degrees and gyro_z={math.degrees(gyro_z):.4f} degrees/s", flush=True)
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
        #w_fused = gyro_z  # or blend gyro + pose
        alpha = 0.5
        w_fused = alpha*gyro_z + (1-alpha)*w_from_pose

        # update history
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        self.prev_time = now

        return v_fused, w_fused

    def path_is_blocked(self, lookahead=30):
        if self.current_path_grid is None:
            return True

        # start checking near where the robot currently is
        xw = -fused_pose['x']
        yw = fused_pose['y']
        rx, ry = self.world_to_grid(xw, yw)

        # find closest point index on the path
        pts = self.current_path_grid
        closest_i = min(range(len(pts)), key=lambda i: (pts[i][0]-rx)**2 + (pts[i][1]-ry)**2)

        end_i = min(len(pts), closest_i + lookahead)

        for (x, y) in pts[closest_i:end_i]:
            if self.grid[y, x] > 1000:
                return True

        return False
    
    def set_goal(self, x_world, y_world):
        print(f"[NAV] New goal set: ({x_world}, {y_world})")

        self.goal_x = float(x_world)
        self.goal_y = float(y_world)

        # Force replan once
        self.prev_goal = None
        


    def robot_far_from_path(self, threshold=0.8):
        if self.current_path_smooth is None or len(self.current_path_smooth) == 0:
            return True
    

        rx = fused_pose['x']
        ry = fused_pose['y']

        min_dist = min(
            math.hypot(rx - px, ry - py)
            for (px, py) in self.current_path_smooth
        )

        return min_dist > threshold

    def plan_to_goal(self, goal_world):
        waypoints = self.test_astar_once(goal_world)

        if waypoints:
            self.current_waypoints = waypoints
            self.current_wp_index = 0
            self.navigation_done = False

    def try_replan(self, goal):
        now = time.time()

        if now - self._last_replan_time < self.REPLAN_COOLDOWN:
            return  # too soon, skip

        print("[REPLAN] Running A*")
        self._last_replan_time = now
        self.plan_to_goal(goal)
        
    def test_astar_once(self, goal_world):
        """
        Minimal A* test.
        - Converts robot pose and goal to grid
        - Inflates obstacles
        - Runs A*
        - Prints result
        No smoothing, no headings, no control yet.
        """
        # ---- 1. Make a binary occupancy map ----
        # ---- 1. Build trinary map ----
        binary = (self.grid > 8).astype(int)

        robot_radius = 0.01  # meters
        inflated = inflate_obstacles(binary, robot_radius, self.cell_size)

        # ---- 3. Convert robot pose to grid ----
        xw = -fused_pose['x']
        yw = fused_pose['y']
        sx, sy = self.world_to_grid(xw, yw)

        # ---- 4. Convert goal to grid ----
        gx, gy = self.world_to_grid(-goal_world[0], goal_world[1])

        # A* expects (x=col, y=row)
        start_A = (sx, sy)
        goal_A  = (gx, gy)

        #print(f"[A* TEST] Start(grid)={start_A}, Goal(grid)={goal_A}")

        # ---- 5. Run A* ----
        path_grid = astar(inflated, start_A, goal_A)

        # Always store the start and goal for visualization
        self.current_astar_start = start_A
        self.current_astar_goal = goal_A

        if path_grid is None:
            print("❌ A* TEST: NO PATH")
            # Clear any previous path data
            self.current_path_grid = None
            self.current_path_world = None
            self.current_path_smooth = None
            return None

        print(f"✅ A* TEST: path returned ({len(path_grid)} points)")

        path_world = grid_to_world(
            path_grid,
            (self.x_origin, self.y_origin),
            self.cell_size
        )

        #print("First few world coords:")
        #for p in path_world[:5]:
        #     print("  ", p)
        #print("Last world coord:", path_world[-1])

        # ---- SMOOTH PATH USING BEZIER ----
        if len(path_world) >= 3:
            # dynamic smoothing resolution
            n_samples = max(40, min(len(path_world) * 5, 300))
            path_smooth = bezier(path_world, n=n_samples)
        else:
            path_smooth = path_world

        decimation_step = 10
        path_smooth = path_smooth[::decimation_step]

        #print("Smoothed path length:", len(path_smooth))

        waypoints = add_headings(path_smooth)

        #print("First 3 waypoints (x, y, theta):")
        #for wp in waypoints[:3]:
        #    print("  ", wp)

        #print("Last waypoint:", waypoints[-1])

        

        

        self.current_waypoints = waypoints

        

        # Store for PG visualizer
        self.current_path_grid = path_grid
        self.current_path_world = path_world
        self.current_path_smooth = path_smooth

   

        return waypoints

       



    

    

   

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

def create_and_run(shared_data, poll=0.005):
    return ObstacleGrid(shared_data, poll=poll)


def main(poll_interval=0.125):  # 100 Hz default
    # Start the shared UART reader (reader-only, no writer)
    import testingUART
    shared_data, running_event, ser = testingUART.start_reader(start_writer=False, do_handshake=False)
    #print("ObstacleGrid: started reader (reader-only). Press Ctrl-C to exit.")
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



