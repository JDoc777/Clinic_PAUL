import sys
import time
import threading
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication
import signal
from Payload import set_flags_byte
import Payload
from testingUART import UARTSharedData
# Import fused_pose from obstacle_grid_processing
from obstacle_grid_processing import fused_pose

def pg_live_plot_loop(grid, update_interval=20):
    """
    Live occupancy grid visualization using pyqtgraph.
    Shows the occupancy grid, the log-odds grid, and the fused robot position (trajectory) side by side.
    Args:
        grid: ObstacleGrid instance
        update_interval: update interval in ms
    """
    app = QApplication.instance() or QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="Occupancy Grid, Log-Odds Grid & Fused Robot Position")
    win.resize(1800, 600)

    # Occupancy Grid Plot (left)
    plot_grid = win.addPlot(title="Occupancy Grid")
    plot_grid.setAspectLocked(True)
    gridDiv = grid.grid.shape[0]
    gridSize = gridDiv * grid.cell_size
    plot_grid.setXRange(-2, 2)
    plot_grid.setYRange(-2, 2)
    plot_grid.showGrid(x=True, y=True, alpha=0.3)
    img = pg.ImageItem()
    plot_grid.addItem(img)
    robot_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_grid.addItem(robot_dot)
    lut = np.zeros((256, 3), dtype=np.ubyte)
    lut[:128] = [0, 255, 0]    # green for free
    lut[128:] = [255, 0, 0]    # red for occupied

    # Log-Odds Grid Plot (middle)
    plot_logodds = win.addPlot(title="Log-Odds Grid")
    plot_logodds.setAspectLocked(True)
    plot_logodds.setXRange(-2, 2)
    plot_logodds.setYRange(-2, 2)
    plot_logodds.showGrid(x=True, y=True, alpha=0.3)
    img_logodds = pg.ImageItem()
    plot_logodds.addItem(img_logodds)

    # --- Add legend for log-odds grid colors ---
    legend_text = (
        "<span style='font-size:12pt'>"
        "<b>Legend:</b><br>"
        "<span style='color:black;'>■</span> Free (log-odds &lt; 0)<br>"
        "<span style='color:gray;'>■</span> Unknown (log-odds ≈ 0)<br>"
        "<span style='color:white;'>■</span> Occupied (log-odds &gt; 0)"
        "</span>"
    )
    legend = pg.TextItem(html=legend_text, anchor=(0,0), border='w', fill=(200, 200, 200, 150))
    plot_logodds.addItem(legend)
    legend.setPos(2.2, -0.5)  # Adjust position as needed

    win.nextRow()  # Move to next row if you want, or comment this out to keep all plots in one row

    # Fused Robot Position Plot (Trajectory) (right)
    plot_traj = win.addPlot(title="Fused Robot Position (Trajectory)")
    plot_traj.setAspectLocked(True)
    plot_traj.setXRange(-2, 2)
    plot_traj.setYRange(-2, 2)
    plot_traj.showGrid(x=True, y=True, alpha=0.3)
    traj_curve = plot_traj.plot(pen=pg.mkPen('b', width=2))
    robot_dot_traj = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255, 0, 0, 200))
    plot_traj.addItem(robot_dot_traj)
    _traj_x = []
    _traj_y = []

    # Add scatter plot for fused obstacle positions
    fused_obs_scatter = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(255, 128, 0, 200))  # Orange
    plot_traj.addItem(fused_obs_scatter)

    # Add scatter plot for sonar tip positions (from sonar_map)
    sonar_tip_scatter = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(0, 255, 255, 200))  # Cyan
    plot_traj.addItem(sonar_tip_scatter)

    # Add lines for fused sonar angles
    sonar_angle_lines = [
        pg.PlotDataItem(pen=pg.mkPen('r', width=2)),  # Front
        pg.PlotDataItem(pen=pg.mkPen('g', width=2)),  # Right
        pg.PlotDataItem(pen=pg.mkPen('b', width=2)),  # Left
        pg.PlotDataItem(pen=pg.mkPen('y', width=2)),  # Back
    ]
    for line in sonar_angle_lines:
        plot_traj.addItem(line)

    def update():
        # Occupancy grid
        grid_disp = np.zeros_like(grid.grid, dtype=np.uint8)
        grid_disp[grid.grid <= 0] = 64   # green
        grid_disp[grid.grid > 0] = 192   # red
        img.setImage(np.flipud(grid_disp.T), levels=(0, 255), lut=lut)
        img.setRect(QtCore.QRectF(-2, -2, 4, 4))  # Fill the plot area

        # Log-odds grid visualization
        logodds_disp = np.clip(grid.grid, -10, 10)
        # Normalize to 0-255 for display: -10 -> 0, 0 -> 127, +10 -> 255
        logodds_img = ((logodds_disp + 10) * (255.0 / 20)).astype(np.uint8)
        img_logodds.setImage(np.flipud(logodds_img.T), levels=(0, 255))
        img_logodds.setRect(QtCore.QRectF(-2, -2, 4, 4))

        # Robot position
        x_pos = fused_pose['x']
        y_pos = fused_pose['y']
        robot_dot.setData([x_pos], [y_pos])

        # Trajectory: only append if position changed
        if not _traj_x or (abs(x_pos - _traj_x[-1]) > 1e-4 or abs(y_pos - _traj_y[-1]) > 1e-4):
            _traj_x.append(x_pos)
            _traj_y.append(y_pos)
        traj_curve.setData(_traj_x, _traj_y)
        robot_dot_traj.setData([x_pos], [y_pos])

        # --- Fused obstacle positions ---
        try:
            sonar_map = grid.sonar_map
            obs_x = [
                sonar_map.F_obstacle_x_fused,
                sonar_map.R_obstacle_x_fused,
                sonar_map.L_obstacle_x_fused,
                sonar_map.B_obstacle_x_fused,
            ]
            obs_y = [
                sonar_map.F_obstacle_y_fused,
                sonar_map.R_obstacle_y_fused,
                sonar_map.L_obstacle_y_fused,
                sonar_map.B_obstacle_y_fused,
            ]
            spots = [{'pos': (x, y), 'brush': (255, 128, 0, 200)} for x, y in zip(obs_x, obs_y)]
            fused_obs_scatter.setData(spots=spots)
        except Exception:
            # If obstacle positions not available yet, skip plotting
            fused_obs_scatter.setData([])

        # --- Fused sonar angles as rays ---
        try:
            sonar_map = grid.sonar_map
            x_pos = fused_pose['x']
            y_pos = fused_pose['y']
            ray_len = 0.5  # meters

            angles = [
                sonar_map.F_sonar_angle_fused,
                sonar_map.R_sonar_angle_fused,
                sonar_map.L_sonar_angle_fused,
                sonar_map.B_sonar_angle_fused,
            ]
            for i, angle in enumerate(angles):
                x_end = x_pos + ray_len * np.cos(angle)
                y_end = y_pos + ray_len * np.sin(angle)
                sonar_angle_lines[i].setData([x_pos, x_end], [y_pos, y_end])
        except Exception:
            # If angles not available, clear the lines
            for line in sonar_angle_lines:
                line.setData([], [])

        # --- Plot sonar tip positions from sonar_map ---
        try:
            sonar_map = grid.sonar_map
            sonar_tip_spots = [
                {'pos': (sonar_map.F_sonar_x_fused, sonar_map.F_sonar_y_fused), 'brush': (0, 255, 255, 200)},
                {'pos': (sonar_map.R_sonar_x_fused, sonar_map.R_sonar_y_fused), 'brush': (0, 255, 255, 200)},
                {'pos': (sonar_map.L_sonar_x_fused, sonar_map.L_sonar_y_fused), 'brush': (0, 255, 255, 200)},
                {'pos': (sonar_map.B_sonar_x_fused, sonar_map.B_sonar_y_fused), 'brush': (0, 255, 255, 200)},
            ]
            sonar_tip_scatter.setData(spots=sonar_tip_spots)
        except Exception:
            sonar_tip_scatter.setData([])

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(update_interval)


    update()  # Initial draw
    


    app.exec_()
