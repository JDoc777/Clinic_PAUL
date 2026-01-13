# ================================================================
# REQUIRED INPUTS FROM PAUL ROBOT (for EKF + Scan Matching SLAM)
# ================================================================
# 1) vx (float) - robot forward velocity [m/s]
#       From encoders: vx = (v_left + v_right) / 2
#
# 2) w (float) - robot angular velocity [rad/s]
#       From IMU gyro: w = gyro_z
#
# 3) x (float) - robot pose x position [m]
#       From local_map.x
#
# 4) y (float) - robot pose y position [m]
#       From local_map.y
#
# 5) theta (float) - robot pose orientation [rad]
#       From local_map.theta (encoder + gyro fused)
#
# 6) sonar_ranges (list of float) - sonar distances [m]
#       Example: [front, front_left, front_right, left]
#       From sonar_processing shared data
#
# 7) sonar_angles (list of float) - sonar mounting angles [rad]
#       Example: np.deg2rad([-30, 0, 30, 90])
#
# 8) occ_grid (2D np.array) - occupancy grid values [0–1]
#       From obstacle_grid.grid
#
# 9) grid_origin (tuple) - world coords of grid cell (0,0)
#       From obstacle_grid.origin_world
#
# 10) cell_size (float) - occupancy grid resolution [m/cell]
#       From obstacle_grid.resolution
#
# These 10 inputs are all that is required for:
#    - EKF prediction step
#    - Scan-to-map alignment (loop closure)
#    - Pose correction and covariance update
# ================================================================



import numpy as np


# ================================================================
# EKF PREDICTOR CLASS
# ================================================================

class EKFPredictor:
    def __init__(self, Q):
        """
        Q: 3x3 process noise matrix
        """
        self.Q = Q

    def predict(self, x_prev, P_prev, v, w, dt):
        """
        EKF motion prediction for unicycle model.
        """
        x, y, theta = x_prev

        # New pose
        x_pred = np.zeros(3)
        x_pred[0] = x + v * dt * np.cos(theta)
        x_pred[1] = y + v * dt * np.sin(theta)
        x_pred[2] = (theta + w * dt + np.pi) % (2*np.pi) - np.pi

        # Jacobian wrt state
        F = np.array([
            [1, 0, -v * dt * np.sin(theta)],
            [0, 1,  v * dt * np.cos(theta)],
            [0, 0,  1]
        ])

        # Covariance update
        P_pred = F @ P_prev @ F.T + self.Q
        return x_pred, P_pred



# ================================================================
# SCAN MATCHING CLASS
# ================================================================

class ScanMatcher:
    def __init__(self, dx=0.05, dy=0.05, dth_deg=5, samples=5):
        """
        dx, dy: search radius in meters
        dth_deg: angular search radius in degrees
        samples: number of samples in each dimension
        """
        self.delta_x = np.linspace(-dx, dx, samples)
        self.delta_y = np.linspace(-dy, dy, samples)
        self.delta_th = np.deg2rad(np.linspace(-dth_deg, dth_deg, samples))

    def match(self, x_r, sonar_ranges, sonar_angles, occ_grid, grid_origin, cell_size):
        """
        Returns corrected pose (x_best, y_best, theta_best).
        """
        x_best = x_r.copy()
        best_score = -np.inf

        h, w = occ_grid.shape
        x0, y0 = grid_origin

        for dx in self.delta_x:
            for dy in self.delta_y:
                for dth in self.delta_th:
                    x_c = x_r[0] + dx
                    y_c = x_r[1] + dy
                    th_c = x_r[2] + dth

                    # Score the sonar hits
                    score = 0.0
                    for r, ang in zip(sonar_ranges, sonar_angles):
                        if r <= 0:
                            continue

                        beam = th_c + ang
                        x_hit = x_c + r * np.cos(beam)
                        y_hit = y_c + r * np.sin(beam)

                        ix = int((x_hit - x0) / cell_size)
                        iy = int((y_hit - y0) / cell_size)

                        if 0 <= ix < w and 0 <= iy < h:
                            score += occ_grid[iy, ix]

                    # Best candidate
                    if score > best_score:
                        best_score = score
                        x_best[:] = [x_c, y_c, th_c]

        return x_best



# ================================================================
# HIGH-LEVEL SLAM SYSTEM CLASS
# ================================================================

class SLAMSystem:
    def __init__(self, Q, dt=0.1):
        self.dt = dt
        self.ekf = EKFPredictor(Q)
        self.matcher = ScanMatcher()

    def update(self, vx, w, x, y, theta,
               sonar_ranges, sonar_angles,
               occ_grid, grid_origin, cell_size,
               P_prev):
        """
        Runs one full SLAM update cycle:
        - EKF prediction
        - scan matching correction
        - covariance shrink
        - confidence weighting
        """

        x_prev = np.array([x, y, theta])

        # ---- EKF PREDICTION ----
        x_pred, P_pred = self.ekf.predict(
            x_prev, P_prev, vx, w, self.dt
        )

        # ---- CONFIDENCE ----
        traceP = np.trace(P_pred)
        confidence = 1.0 / (1.0 + traceP)

        # ---- SCAN MATCH ----
        x_corr = self.matcher.match(
            x_pred, sonar_ranges, sonar_angles,
            occ_grid, grid_origin, cell_size
        )

        # ---- REDUCE UNCERTAINTY AFTER CORRECTION ----
        P_corr = P_pred * 0.7

        # Return corrected pose, covariance, confidence
        return (x_corr[0], x_corr[1], x_corr[2]), P_corr, confidence



# ================================================================
# EXAMPLE SIMULATION (same as before but using classes)
# ================================================================

if __name__ == "__main__":
    dt = 0.1
    Q = np.diag([0.002, 0.002, 0.001])

    slam = SLAMSystem(Q, dt)

    # Initial pose and covariance
    x, y, theta = 0.0, 0.0, 0.0
    P = np.eye(3)*0.01

    # Fake inputs
    sonar_angles = np.deg2rad([0, -90, 90, 180])
    sonar_ranges = [0.5, 0.7, 0.6, 0.8]
    occ_grid = np.random.rand(200, 200)
    grid_origin = (0.0, 0.0)
    cell_size = 0.05

    # Run example
    for step in range(50):
        vx = 0.2 + np.random.randn()*0.01
        w  = 0.05 + np.random.randn()*0.01

        (x, y, theta), P, conf = slam.update(
            vx, w, x, y, theta,
            sonar_ranges, sonar_angles,
            occ_grid, grid_origin, cell_size,
            P
        )

    print("Final pose:", x, y, theta)
    print("Final covariance:\n", P)
    print("Final confidence:", conf)


'''
    # INPUTS from PAUL
    vx = encoder_processor.forward_velocity      # m/s
    w  = imu_processor.gyro_z                   # rad/s

    x  = local_map.x
    y  = local_map.y
    theta = local_map.theta

    sonar_ranges = [
        shared.front,
        shared.front_left,
        shared.front_right,
        shared.left
    ]

    sonar_angles = np.deg2rad([-30, 0, 30, 90])

    occ_grid = obstacle_grid.grid
    grid_origin = obstacle_grid.origin_world
    cell_size   = obstacle_grid.resolution

    # ---- SLAM Update ----
    corrected_x, corrected_y, corrected_theta, P, confidence = slam_update(
        vx, w, x, y, theta,
        sonar_ranges,
        sonar_angles,
        occ_grid,
        grid_origin,
        cell_size
    )

    # ---- USE OUTPUTS ----

    # (1) Update global pose
    local_map.x = corrected_x
    local_map.y = corrected_y
    local_map.theta = corrected_theta

    # (2) Apply confidence weighting to occupancy grid update
    obstacle_grid.update(
        sonar_ranges,
        corrected_x,
        corrected_y,
        corrected_theta,
        weight=confidence
    )

    # (3) (Optional) Visualize ellipse
    # plot_covariance_ellipse(corrected_x, corrected_y, P)
'''