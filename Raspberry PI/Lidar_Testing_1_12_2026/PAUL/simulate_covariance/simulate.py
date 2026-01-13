"""
simulate.py
Runs a live PyQtGraph simulation using the SLAMSystem from test.py

Features:
- Fake robot motion (vx, w)
- 4 sonar sensors (front, left, right, back)
- Fake obstacles
- Log-odds occupancy grid update
- Real-time visualization of:
    * Robot path
    * Sonar beams
    * Occupancy grid
    * SLAM-corrected robot pose
"""

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore  # Update QtGui to QtWidgets
import threading  # Add threading module
import time  # Import time for sleep


from test import SLAMSystem  # <-- IMPORT YOUR SLAM CLASSES


# ============================================================
# PARAMETERS
# ============================================================
DT = 0.1
MAP_SIZE = 200
CELL_SIZE = 0.05  # 5cm
WORLD_SIZE = MAP_SIZE * CELL_SIZE

# Log-odds parameters
L_FREE = -0.3
L_OCC = +0.4
L_MIN = -5
L_MAX = +5


# ============================================================
# BUILD FAKE OCCUPANCY GRID (GROUND TRUTH)
# ============================================================
true_grid = np.zeros((MAP_SIZE, MAP_SIZE))

# Add some random obstacles
for _ in range(15):
    cx = np.random.randint(20, MAP_SIZE-20)
    cy = np.random.randint(20, MAP_SIZE-20)
    w  = np.random.randint(4, 10)
    h  = np.random.randint(4, 10)
    true_grid[cy:cy+h, cx:cx+w] = 1.0


# ============================================================
# MEASURE SONAR RANGE
# ============================================================
def fake_sonar(x, y, theta, angle):
    """
    Raycast into the true grid.
    """
    max_range = 2.0
    steps = int(max_range / CELL_SIZE)

    beam_ang = theta + angle

    for i in range(steps):
        dist = i * CELL_SIZE
        px = x + dist * np.cos(beam_ang)
        py = y + dist * np.sin(beam_ang)

        ix = int(px / CELL_SIZE)
        iy = int(py / CELL_SIZE)

        if ix < 0 or ix >= MAP_SIZE or iy < 0 or iy >= MAP_SIZE:
            return dist

        if true_grid[iy, ix] > 0.5:
            return dist

    return max_range


# ============================================================
# LOG-ODDS UPDATE
# ============================================================
def update_log_odds(log_odds, x, y, theta, sonar_ranges, sonar_angles):
    for r, ang in zip(sonar_ranges, sonar_angles):
        beam_ang = theta + ang

        steps = int(r / CELL_SIZE)
        for i in range(steps):
            px = x + i*CELL_SIZE*np.cos(beam_ang)
            py = y + i*CELL_SIZE*np.sin(beam_ang)
            ix = int(px / CELL_SIZE)
            iy = int(py / CELL_SIZE)

            if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
                log_odds[iy, ix] += L_FREE

        # Mark hit cell as occupied
        hx = x + r*np.cos(beam_ang)
        hy = y + r*np.sin(beam_ang)
        ix = int(hx / CELL_SIZE)
        iy = int(hy / CELL_SIZE)
        if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
            log_odds[iy, ix] += L_OCC

    # Clamp
    log_odds[:] = np.clip(log_odds, L_MIN, L_MAX)
    return log_odds


# ============================================================
# PYQTGRAPH WINDOW SETUP (TWO MAPS)
# ============================================================
app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="SLAM Simulation")
win.resize(1200, 600)

# Left view: Original robot position map
view_left = win.addViewBox(row=0, col=0)
view_left.setAspectLocked(True)
img_left = pg.ImageItem()
view_left.addItem(img_left)

# Right view: Confidence-weighted map
view_right = win.addViewBox(row=0, col=1)
view_right.setAspectLocked(True)
img_right = pg.ImageItem()
view_right.addItem(img_right)

# Robot path plot (left map)
path_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
view_left.addItem(path_curve)

# Sonar beams (left map)
sonar_lines = [pg.PlotDataItem(pen=pg.mkPen('y', width=2)) for _ in range(4)]
for s in sonar_lines:
    view_left.addItem(s)


# ============================================================
# INITIAL CONDITIONS
# ============================================================
slam = SLAMSystem(Q=np.diag([0.002, 0.002, 0.001]), dt=DT)

x, y, theta = 2.0, 2.0, 0.0
P = np.eye(3)*0.01

path_x = []
path_y = []

log_odds = np.zeros((MAP_SIZE, MAP_SIZE))

sonar_angles = np.deg2rad([0, -90, 90, 180])


# ============================================================
# ADD RANDOM CIRCULAR OBSTACLES
# ============================================================
def add_random_circle(grid, radius_range=(2, 5)):
    """
    Adds a random circular obstacle to the grid.
    """
    radius = np.random.randint(*radius_range)
    cx = np.random.randint(radius, MAP_SIZE - radius)
    cy = np.random.randint(radius, MAP_SIZE - radius)

    y, x = np.ogrid[-radius:radius, -radius:radius]
    mask = x**2 + y**2 <= radius**2

    grid[cy-radius:cy+radius, cx-radius:cx+radius][mask] = 1.0


# ============================================================
# MAIN UPDATE LOOP (THREADED)
# ============================================================
def update():
    global x, y, theta, P, log_odds

    # Add variables to control motion randomness
    vx_base = 0.5  # Base forward velocity
    w_base = 0.2   # Base angular velocity

    # Add a timer for placing random obstacles
    obstacle_timer = 0
    obstacle_interval = 50  # Add a new obstacle every 50 iterations

    try:
        while True:  # Continuous loop for threading
            # Introduce random variations in motion
            vx = vx_base + np.random.uniform(-0.2, 0.2)  # Random forward velocity
            w = w_base + np.random.uniform(-0.1, 0.1)    # Random angular velocity

            # Update robot pose based on motion model
            x += vx * DT * np.cos(theta)
            y += vx * DT * np.sin(theta)
            theta += w * DT
            theta = (theta + np.pi) % (2 * np.pi) - np.pi  # Normalize angle to [-pi, pi]

            # Ensure the robot stays within the map boundaries
            x = np.clip(x, 0.1, WORLD_SIZE - 0.1)
            y = np.clip(y, 0.1, WORLD_SIZE - 0.1)

            # Fake sonar ranges (raycast into true grid)
            sonar_ranges = [fake_sonar(x, y, theta, a) for a in sonar_angles]

            # SLAM update
            (x, y, theta), P, conf = slam.update(
                vx, w, x, y, theta,
                sonar_ranges,
                sonar_angles,
                log_odds_to_prob(log_odds),  # for scan matching
                (0.0, 0.0),
                CELL_SIZE,
                P
            )

            # Update occupancy grid log-odds
            log_odds = update_log_odds(log_odds, x, y, theta, sonar_ranges, sonar_angles)

            # Plot original occupancy grid (left map)
            img_left.setImage(log_odds_to_prob(log_odds).T)

            # Plot confidence-weighted map (right map)
            weighted_log_odds = log_odds * conf
            img_right.setImage(log_odds_to_prob(weighted_log_odds).T)

            # Save path
            path_x.append(x)
            path_y.append(y)
            path_curve.setData(path_x, path_y)

            # Update sonar beams (left map)
            for i, (rng, ang) in enumerate(zip(sonar_ranges, sonar_angles)):
                bx = x + rng * np.cos(theta + ang)
                by = y + rng * np.sin(theta + ang)
                sonar_lines[i].setData([x, bx], [y, by])

            # Add random circular obstacles at intervals
            obstacle_timer += 1
            if obstacle_timer >= obstacle_interval:
                add_random_circle(true_grid)
                obstacle_timer = 0

            time.sleep(DT)  # Sleep for DT seconds
    except Exception as e:
        print(f"Error in update thread: {e}")
        raise


def log_odds_to_prob(L):
    return 1 - 1/(1 + np.exp(L))


# ============================================================
# START THREAD AND HANDLE CTRL+C
# ============================================================
def main():
    try:
        # Start the update thread
        update_thread = threading.Thread(target=update, daemon=True)
        update_thread.start()

        # Start the PyQtGraph application
        QtWidgets.QApplication.instance().exec()  # Replace exec_() with exec()
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing application...")
        QtWidgets.QApplication.instance().quit()  # Gracefully quit the application
        app.closeAllWindows()  # Close all PyQtGraph windows
        raise  # Re-raise the exception to exit the program

if __name__ == "__main__":
    main()