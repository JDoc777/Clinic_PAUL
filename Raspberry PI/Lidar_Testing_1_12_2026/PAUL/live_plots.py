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
from reverse_kinematics import ReverseKinematics
from lidar_local_map import LidarLocalMap
from lidar_obstacle_map import LidarObstacleMap

from LocomotionSIM2 import MAP_H_M, MAP_W_M, build_section2_segmented_path_from_smooth_path, slope_intersection



# Set the background color to white
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')  # Set foreground (text, grid lines) to black


def pg_live_plot_loop(grid, sec3=None, update_interval=10, servo_controller=None):

    def world_to_grid_coords(grid, x_world, y_world):
        ix = int((x_world - grid.x_origin) / grid.cell_size)
        iy = int((y_world - grid.y_origin) / grid.cell_size)
        return ix, iy

    """
    Live occupancy grid visualization using pyqtgraph.
    Shows the occupancy grid, the log-odds grid, the fused robot position (trajectory), and servo angles.
    Args:
        grid: ObstacleGrid instance
        update_interval: update interval in ms
        servo_controller: ServoController instance to fetch servo angles
    """
    app = QApplication.instance() or QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="Occupancy Grid, Log-Odds Grid, Fused Robot Position & Servo Angles")
    win.resize(1800, 800)


    # Log-Odds Grid Plot (top-right)
    plot_logodds = win.addPlot(title="Log-Odds Grid")
    plot_logodds.setAspectLocked(True)
    plot_logodds.setXRange(0, grid.grid.shape[1])
    plot_logodds.setYRange(0, grid.grid.shape[0])
    plot_logodds.showGrid(x=True, y=True, alpha=0.3)
    img_logodds = pg.ImageItem()
    plot_logodds.addItem(img_logodds)

    # Add robot position dot to log-odds plot
    robot_dot_logodds = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_logodds.addItem(robot_dot_logodds)

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

    # Corrected Positions Plot (top-right)
    plot_corrected = win.addPlot(title="Corrected Positions (Fused)")
    plot_corrected.setAspectLocked(True)
    plot_corrected.setXRange(0, grid.grid.shape[1])
    plot_corrected.setYRange(0, grid.grid.shape[0])
    plot_corrected.showGrid(x=True, y=True, alpha=0.3)
    img_corrected = pg.ImageItem()
    plot_corrected.addItem(img_corrected)

    # Add robot position dot to corrected positions plot
    robot_dot_corrected = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_corrected.addItem(robot_dot_corrected)

    # Add legend for corrected positions
    corrected_legend_text = (
        "<span style='font-size:12pt'>"
        "<b>Legend:</b><br>"
        "<span style='color:black;'>■</span> Free (log-odds &lt; 0)<br>"
        "<span style='color:gray;'>■</span> Unknown (log-odds ≈ 0)<br>"
        "<span style='color:white;'>■</span> Occupied (log-odds &gt; 0)"
        "</span>"
    )
    corrected_legend = pg.TextItem(html=corrected_legend_text, anchor=(0, 0), border='w', fill=(200, 200, 200, 150))
    plot_corrected.addItem(corrected_legend)
    corrected_legend.setPos(2.2, -0.5)  # Adjust position as needed


    # Move to the next row for the remaining bottom plots
    win.nextRow()

    # Fused Robot Position Plot (Trajectory) (bottom-left)
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








    # --- Wheel Speed Bars ---
    plot_wheels = win.addPlot(title="Wheel Speeds")
    plot_wheels.setYRange(-1.5, 1.5)
    plot_wheels.setXRange(-0.5, 3.5)
    plot_wheels.showGrid(x=True, y=True, alpha=0.3)

    wheel_names = ["FL", "FR", "BL", "BR"]

    wheel_bars = pg.BarGraphItem(
        x=np.arange(4),
        height=[0, 0, 0, 0],
        width=0.6,
        brushes=['r', 'g', 'b', 'y']
    )
    plot_wheels.addItem(wheel_bars)

    wheel_labels = []
    for i in range(4):
        label = pg.TextItem(text="", anchor=(0.5, -0.5), color='k')
        plot_wheels.addItem(label)
        wheel_labels.append(label)

    axis = plot_wheels.getAxis('bottom')
    axis.setTicks([[(i, wheel_names[i]) for i in range(4)]])




    # Binary Map Plot with A* Path
    plot_binary = win.addPlot(title="Binary Map with A* Path")
    plot_binary.setAspectLocked(True)
    plot_binary.setXRange(0, grid.grid.shape[1])
    plot_binary.setYRange(0, grid.grid.shape[0])
    plot_binary.showGrid(x=True, y=True, alpha=0.3)
    img_binary = pg.ImageItem()
    plot_binary.addItem(img_binary)

    # Add items for A* visualization on the binary map
    astar_start_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 255, 0, 200))  # Green for start
    astar_goal_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255, 0, 0, 200))  # Red for goal
    astar_raw_path = pg.PlotDataItem(pen=pg.mkPen('y', width=2))  # Yellow for raw path
    astar_smooth_path = pg.PlotDataItem(pen=pg.mkPen('r', width=2))  # Red for smoothed path

    plot_binary.addItem(astar_start_dot)
    plot_binary.addItem(astar_goal_dot)
    plot_binary.addItem(astar_raw_path)
    plot_binary.addItem(astar_smooth_path)

        # ---------------- LiDAR Scan Plot (Robot Frame) ----------------
    '''plot_lidar = win.addPlot(title="LiDAR Scan (Robot Frame)")
    plot_lidar.setAspectLocked(True)
    plot_lidar.setXRange(-3, 3)
    plot_lidar.setYRange(-3, 3)
    plot_lidar.showGrid(x=True, y=True, alpha=0.3)

    lidar_scatter = pg.ScatterPlotItem(size=3, brush=pg.mkBrush(120, 0, 255, 160))  # purple-ish
    plot_lidar.addItem(lidar_scatter)

    # robot origin marker
    lidar_origin = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 0, 200))
    lidar_origin.setData([0.0], [0.0])
    plot_lidar.addItem(lidar_origin)'''

        # --- LiDAR Local Map Plot ---
    plot_lidar = win.addPlot(title="LiDAR Local Map")
    plot_lidar.setAspectLocked(True)
    plot_lidar.setXRange(-3, 3)
    plot_lidar.setYRange(-3, 3)
    plot_lidar.showGrid(x=True, y=True, alpha=0.3)
        # --- Robot trajectory on LiDAR plot ---
    lidar_traj_curve = plot_lidar.plot(pen=pg.mkPen('b', width=2))
    lidar_traj_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_lidar.addItem(lidar_traj_dot)

    lidar_traj_x = []
    lidar_traj_y = []

    lidar_scatter = pg.ScatterPlotItem(
        size=3,
        brush=pg.mkBrush(255, 0, 255, 180)  # magenta
    )
    plot_lidar.addItem(lidar_scatter)

        # Robot position dot (LiDAR plot)
    lidar_robot_dot = pg.ScatterPlotItem(
        size=12,
        brush=pg.mkBrush(0, 0, 255, 200)  # blue
    )
    plot_lidar.addItem(lidar_robot_dot)

    # ===== SECTION 3: STATE MACHINE (MID RIGHT) =====
    plot_sec3 = win.addPlot(title="Section 3: State Machine")
    plot_sec3.setXRange(0, MAP_W_M)
    plot_sec3.setYRange(0, MAP_H_M)
    plot_sec3.setAspectLocked(True)
    plot_sec3.showGrid(x=True, y=True, alpha=0.2)

    # robot dot
    sec3_robot = plot_sec3.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolSize=10)

    # heading line
    sec3_heading = plot_sec3.plot([], [], pen=pg.mkPen('y', width=3))

    sec3_path = plot_sec3.plot([], [], pen=pg.mkPen('m', width=3))

    # text
    sec3_text = pg.TextItem("", anchor=(0, 0), color='k')
    plot_sec3.addItem(sec3_text)



    win.nextRow()  # Move to the next row for the Section 2 plots

    plot_sec2_analysis = win.addPlot(title="Section 2: Path Analysis")
    plot_sec2_analysis.setXRange(0, 5)
    plot_sec2_analysis.setYRange(0, 5)
    plot_sec2_analysis.setAspectLocked(True)
    plot_sec2_analysis.showGrid(x=True, y=True, alpha=0.2)


    analysis_smooth = plot_sec2_analysis.plot(pen=pg.mkPen('r', width=3))

    analysis_blue = pg.ScatterPlotItem(size=10, brush='b')
    analysis_orange = pg.ScatterPlotItem(size=10, brush=(255, 165, 0))
    analysis_purple = pg.ScatterPlotItem(size=10, brush=(128, 0, 128))

    plot_sec2_analysis.addItem(analysis_blue)
    plot_sec2_analysis.addItem(analysis_orange)
    plot_sec2_analysis.addItem(analysis_purple)

    analysis_labels = []

    plot_sec2_turn = win.addPlot(title="Section 2: Turning Point Calculation")
    plot_sec2_turn.setXRange(0, 5)
    plot_sec2_turn.setYRange(0, 5)
    plot_sec2_turn.setAspectLocked(True)
    plot_sec2_turn.showGrid(x=True, y=True, alpha=0.2)

    turn_smooth = plot_sec2_turn.plot(pen=pg.mkPen('r', width=3))
    turn_items = []   # IMPORTANT: store ALL dynamic items

    plot_sec2_modified = win.addPlot(title="Section 2: Modified Path")
    plot_sec2_modified.setXRange(0, 5)
    plot_sec2_modified.setYRange(0, 5)
    plot_sec2_modified.setAspectLocked(True)
    plot_sec2_modified.showGrid(x=True, y=True, alpha=0.2)

    modified_curve = plot_sec2_modified.plot(pen=pg.mkPen('b', width=3))
    modified_items = []

    plot_sec2_mode = win.addPlot(title="Section 2: Path Segmentation")
    plot_sec2_mode.setXRange(0, 5)
    plot_sec2_mode.setYRange(0, 5)
    plot_sec2_mode.setAspectLocked(True)
    plot_sec2_mode.showGrid(x=True, y=True, alpha=0.2)

    mode_curve = plot_sec2_mode.plot(pen=pg.mkPen('g', width=3))
    mode_items = []

    

    

   






    def update():
        QtCore.QCoreApplication.processEvents()

        # Log-odds grid visualization
        logodds_disp = np.clip(grid.grid, -10, 10)
        # Normalize to 0-255 for display: -10 -> 0, 0 -> 127, +10 -> 255
        logodds_img = ((logodds_disp + 10) * (255.0 / 20)).astype(np.uint8)
        img_logodds.setImage(np.flipud(logodds_img.T), levels=(0, 255))
        img_logodds.setRect(QtCore.QRectF(0, 0, grid.grid.shape[1], grid.grid.shape[0]))

        # Corrected positions visualization
        try:
            # Ensure corrected_grid is properly normalized and visualized
            corrected_disp = np.clip(grid.grid, -10, 10)  # Use the same grid data as log-odds for now
            # Normalize to 0-255 for display: -10 -> 0, 0 -> 127, +10 -> 255
            corrected_img = ((corrected_disp + 10) * (255.0 / 20)).astype(np.uint8)
            img_corrected.setImage(np.flipud(corrected_img.T), levels=(0, 255))
            img_corrected.setRect(QtCore.QRectF(0, 0, grid.grid.shape[1], grid.grid.shape[0]))
        except Exception:
            # If corrected positions not available yet, skip plotting
            pass

        # Robot position
        x_pos = fused_pose['x']
        y_pos = fused_pose['y']
        robot_ix, robot_iy = world_to_grid_coords(grid, x_pos, y_pos)

        #robot_dot.setData([robot_ix], [robot_iy])
        robot_dot_logodds.setData([robot_ix], [robot_iy])
        robot_dot_corrected.setData([robot_ix], [robot_iy])

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

    
                # ---- Wheel Speed Plot ----
        try:
            rk = grid.shared.reverse_kinematics

            # DEBUG: Show if plot is reading correct RK instance
        

            wheel_speeds = [rk.FL, rk.FR, rk.BL, rk.BR]

            wheel_bars.setOpts(height=wheel_speeds)

            for i, lbl in enumerate(wheel_labels):
                ws = wheel_speeds[i]
                lbl.setText(f"{ws:.2f}")
                lbl.setPos(i, ws + (0.1 if ws >= 0 else -0.1))

        except Exception as e:
            #print("Wheel plot error:", e)
            pass
            

        # Binary map visualization
        binary_map = (grid.grid >= 1).astype(np.uint8)
        binary_img = (binary_map * 255).astype(np.uint8)  # Scale binary values to 0-255
        #img_binary.setImage(np.flipud(binary_img.T), levels=(0, 255))
        img_binary.setImage(binary_img.T, levels=(0, 255))
        img_binary.setRect(QtCore.QRectF(0, 0, grid.grid.shape[1], grid.grid.shape[0]))

        # A* Pathfinding Visualization on Binary Map
        try:
            # Start and goal positions
            start = grid.current_astar_start
            goal = grid.current_astar_goal
            astar_start_dot.setData([start[0]], [start[1]])
            astar_goal_dot.setData([goal[0]], [goal[1]])

            # Raw A* path
            if grid.current_path_grid:
                px = [p[0] for p in grid.current_path_grid]
                py = [p[1] for p in grid.current_path_grid]
                astar_raw_path.setData(px, py)

            # Smoothed path
            if grid.current_path_smooth is not None and grid.current_path_smooth.size > 0:
                sx = [(p[0] - grid.x_origin) / grid.cell_size for p in grid.current_path_smooth]
                sy = [(p[1] - grid.y_origin) / grid.cell_size for p in grid.current_path_smooth]
                astar_smooth_path.setData(sx, sy)
        except AttributeError:
            # If A* data is not available yet, clear the plot
            astar_start_dot.setData([], [])
            astar_goal_dot.setData([], [])
            astar_raw_path.setData([], [])
            astar_smooth_path.setData([], [])
        
            # ---------------- LiDAR Scan (Robot Frame) ----------------
        '''try:
            scan = None
            if hasattr(grid, "lidar_proc") and grid.lidar_proc is not None:
                scan = grid.lidar_proc.get_latest_scan()

            if scan is not None and scan.ranges_m.size > 0:
                ang = scan.angles_rad
                rng = scan.ranges_m

                # Optional downsample so plot is fast
                step = 6
                ang = ang[::step]
                rng = rng[::step]

                # Convert polar -> xy in robot frame
                x = rng * np.cos(ang)
                y = rng * np.sin(ang)

                spots = [{'pos': (float(xi), float(yi))} for xi, yi in zip(x, y)]
                lidar_scatter.setData(spots=spots)
            else:
                lidar_scatter.setData([])
        except Exception:
            lidar_scatter.setData([])
        '''
        # --- LiDAR local map points ---
        try:
            lidar_map = grid.lidar_map
            pts = lidar_map.points_world

            if pts:
                xs, ys = zip(*pts)
                lidar_scatter.setData(xs, ys)
            else:
                lidar_scatter.setData([], [])

        except Exception:
            lidar_scatter.setData([], [])

        # --- Robot position on LiDAR plot ---
        try:
            x = fused_pose['x']
            y = fused_pose['y']
            lidar_robot_dot.setData([x], [y])
            # Append trajectory if moved
            if not lidar_traj_x or (abs(x - lidar_traj_x[-1]) > 1e-4 or abs(y - lidar_traj_y[-1]) > 1e-4):
                lidar_traj_x.append(x)
                lidar_traj_y.append(y)

            lidar_traj_curve.setData(lidar_traj_x, lidar_traj_y)
            lidar_traj_dot.setData([x], [y])
        except Exception:
            lidar_robot_dot.setData([], [])



                # -------- Section 2 DATA COMPUTATION --------
        sec2_data = None

        try:
            smooth_path = getattr(grid, "current_path_smooth", None)
            raw_path = getattr(grid, "current_path_world", None)

            if smooth_path is not None and len(smooth_path) >= 3:

                smooth_np = np.asarray(smooth_path, dtype=float)

                if raw_path is None:
                    raw_np = smooth_np
                else:
                    raw_np = np.asarray(raw_path, dtype=float)

                sec2_data = build_section2_segmented_path_from_smooth_path(
                    smooth_path_m=smooth_np,
                    raw_path_m=raw_np,
                    grid=np.copy(grid.grid),
                    start_m=smooth_np[0],
                    goal_m=smooth_np[-1]
                )

        except Exception as e:
            print("[SEC2 compute error]:", e)

        # -------- Section 2: Path Analysis --------
        raw_np = None
        try:

            # clear previous radius labels
            for lbl in analysis_labels:
                plot_sec2_analysis.removeItem(lbl)
            analysis_labels.clear()

            # smooth path (red)
            if sec2_data is not None and "smooth_path" in sec2_data:
                sp = np.asarray(sec2_data["smooth_path"], dtype=float)
                analysis_smooth.setData(sp[:, 0], sp[:, 1])
            else:
                analysis_smooth.setData([], [])

            blue_pts = []
            orange_pts = []
            purple_pts = []

            if sec2_data is not None and "turning_points" in sec2_data:

                for tp in sec2_data["turning_points"].values():

                    start_point = tp["start"]["point"]
                    start_R = tp["start"]["R"]
                    intersection = tp["start"]["intersection"]

                    blue_pts.append({
                        "pos": (float(start_point[0]), float(start_point[1]))
                    })

                    orange_pts.append({
                        "pos": (float(intersection[0]), float(intersection[1]))
                    })

                    label = pg.TextItem(
                        text=f"R={start_R:.2f}",
                        color='k',
                        anchor=(0, 1)
                    )
                    label.setPos(
                        float(start_point[0] + 0.08),
                        float(start_point[1] + 0.08)
                    )
                    plot_sec2_analysis.addItem(label)
                    analysis_labels.append(label)

                    if "end" in tp:
                        end_point = tp["end"]["point"]
                        purple_pts.append({
                            "pos": (float(end_point[0]), float(end_point[1]))
                        })

            analysis_blue.setData(spots=blue_pts)
            analysis_orange.setData(spots=orange_pts)
            analysis_purple.setData(spots=purple_pts)

        except Exception as e:
            print("[SEC2 path analysis error]:", e)

            analysis_smooth.setData([], [])
            analysis_blue.setData([])
            analysis_orange.setData([])
            analysis_purple.setData([])

        # -------- Section 2: Turning Point Calculation --------
        try:

            # clear previous items (IMPORTANT)
            for item in turn_items:
                plot_sec2_turn.removeItem(item)
            turn_items.clear()

            if sec2_data is not None:

                sp = np.asarray(sec2_data["smooth_path"], dtype=float)
                turn_smooth.setData(sp[:, 0], sp[:, 1])

                color_list = [
                    (31,119,180),(255,127,14),(44,160,44),(214,39,40),
                    (148,103,189),(140,86,75),(227,119,194),(127,127,127)
                ]

                slope_len = 0.5

                for idx, (_, tp) in enumerate(sec2_data["turning_points"].items()):

                    color = color_list[idx % len(color_list)]
                    pen = pg.mkPen(color, width=2)

                    # -------- START POINT --------
                    start_pt = tp["start"]["point"]
                    start_slope = tp["start"]["slope"]
                    x0, y0 = start_pt

                    pt = pg.ScatterPlotItem(
                        [x0], [y0],
                        size=10,
                        brush=color
                    )
                    plot_sec2_turn.addItem(pt)
                    turn_items.append(pt)

                    # START SLOPE LINE
                    if np.isinf(start_slope):
                        x = [x0, x0]
                        y = [y0, y0 + slope_len]
                    else:
                        dx = slope_len
                        dy = start_slope * dx
                        x = [x0, x0 + dx]
                        y = [y0, y0 + dy]

                    line = pg.PlotDataItem(x, y, pen=pen)
                    plot_sec2_turn.addItem(line)
                    turn_items.append(line)

                    # -------- END POINT --------
                    if "end" in tp:

                        end_pt = tp["end"]["point"]
                        end_slope = tp["end"]["slope"]
                        x1, y1 = end_pt

                        pt2 = pg.ScatterPlotItem(
                            [x1], [y1],
                            size=10,
                            brush=color
                        )
                        plot_sec2_turn.addItem(pt2)
                        turn_items.append(pt2)

                        # END SLOPE (BACKWARDS!)
                        if np.isinf(end_slope):
                            x = [x1, x1]
                            y = [y1, y1 - slope_len]
                        else:
                            dx = -slope_len
                            dy = end_slope * dx
                            x = [x1, x1 + dx]
                            y = [y1, y1 + dy]

                        line2 = pg.PlotDataItem(x, y, pen=pen)
                        plot_sec2_turn.addItem(line2)
                        turn_items.append(line2)

                        # -------- INTERSECTION (YELLOW) --------
                        inter = slope_intersection(
                            start_pt, start_slope,
                            end_pt, end_slope
                        )

                        if inter is not None:
                            ix, iy = inter

                            ipt = pg.ScatterPlotItem(
                                [ix], [iy],
                                size=9,
                                brush='y'
                            )
                            plot_sec2_turn.addItem(ipt)
                            turn_items.append(ipt)

            else:
                turn_smooth.setData([], [])

        except Exception as e:
            print("[SEC2 turn error]:", e)
            turn_smooth.setData([], [])

    # -------- Section 2: Modified Path --------
        try:

            # clear previous items
            for item in modified_items:
                plot_sec2_modified.removeItem(item)
            modified_items.clear()

            if sec2_data is not None and "modified_path" in sec2_data:

                mp = np.asarray(sec2_data["modified_path"], dtype=float)
                modified_curve.setData(mp[:, 0], mp[:, 1])

                for tp in sec2_data["turning_points"].values():

                    if "end" not in tp:
                        continue

                    start_pt = tp["start"]["point"]
                    start_slope = tp["start"]["slope"]

                    end_pt = tp["end"]["point"]
                    end_slope = tp["end"]["slope"]

                    inter = slope_intersection(
                        start_pt, start_slope,
                        end_pt, end_slope
                    )

                    if inter is None:
                        continue

                    ix, iy = inter

                    pt = pg.ScatterPlotItem(
                        [ix], [iy],
                        size=10,
                        brush='y'
                    )
                    plot_sec2_modified.addItem(pt)
                    modified_items.append(pt)

            else:
                modified_curve.setData([], [])

        except Exception as e:
            print("[SEC2 modified error]:", e)
            modified_curve.setData([], [])

    # -------- Section 2: Path Segmentation --------
        try:

            # clear previous items
            for item in mode_items:
                plot_sec2_mode.removeItem(item)
            mode_items.clear()

            if sec2_data is not None and "modified_path" in sec2_data:

                mp = np.asarray(sec2_data["modified_path"], dtype=float)
                mode_curve.setData(mp[:, 0], mp[:, 1])

                last_tip_index = 0
                tip_i = 1

                for inter in sec2_data["turn_intersections"]:

                    if len(mp) == 0:
                        break

                    distances = np.linalg.norm(mp - inter, axis=1)
                    tip_index = int(np.argmin(distances))

                    mid_index = (last_tip_index + tip_index) // 2
                    p_mid = mp[mid_index]

                    # ACK label (green)
                    ack = pg.TextItem("ACK", color=(0, 100, 0))
                    ack.setPos(float(p_mid[0]), float(p_mid[1]))
                    plot_sec2_mode.addItem(ack)
                    mode_items.append(ack)

                    # TIP label (orange)
                    tip_label = pg.TextItem(f"TIP{tip_i}", color=(255, 140, 0))
                    tip_label.setPos(float(inter[0]), float(inter[1] + 0.08))
                    plot_sec2_mode.addItem(tip_label)
                    mode_items.append(tip_label)

                    # TIP point
                    tip_pt = pg.ScatterPlotItem(
                        [inter[0]], [inter[1]],
                        size=11,
                        brush=(255, 165, 0)
                    )
                    plot_sec2_mode.addItem(tip_pt)
                    mode_items.append(tip_pt)

                    last_tip_index = tip_index
                    tip_i += 1

                # final ACK
                if len(mp) > 0:
                    mid_index = (last_tip_index + len(mp) - 1) // 2
                    p_mid = mp[mid_index]

                    ack = pg.TextItem("ACK", color=(0, 100, 0))
                    ack.setPos(float(p_mid[0]), float(p_mid[1]))
                    plot_sec2_mode.addItem(ack)
                    mode_items.append(ack)

            else:
                mode_curve.setData([], [])

        except Exception as e:
            print("[SEC2 mode error]:", e)
            mode_curve.setData([], [])
        
        # ===== SECTION 3 VISUAL =====
        try:
            if sec3 is not None:
                # draw path (same as segmentation)
                if sec2_data is not None and "modified_path" in sec2_data:
                    mp = np.asarray(sec2_data["modified_path"], dtype=float)
                    sec3_path.setData(mp[:, 0], mp[:, 1])
                else:
                    sec3_path.setData([], [])

                rx, ry = sec3.render_pos
                theta = sec3.render_theta

                # robot dot
                sec3_robot.setData([rx], [ry])

                # heading line
                hx = rx + 0.05 * np.cos(theta)
                hy = ry + 0.05 * np.sin(theta)
                sec3_heading.setData([rx, hx], [ry, hy])

                # text (same style as SIM2)
                sec3_text.setText(sec3.render_text)
                sec3_text.setPos(rx + 0.1, ry + 0.1)

        except Exception as e:
            # don't crash GUI
            pass

        

        
            

        

               

        
    





    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(update_interval)


    update()  # Initial draw
    
    app.exec_()

