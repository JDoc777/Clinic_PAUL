import sys
import time
import threading
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QGraphicsProxyWidget
import signal
from Payload import set_flags_byte
import Payload
from testingUART import UARTSharedData
# Import fused_pose from obstacle_grid_processing
from obstacle_grid_processing import fused_pose
from reverse_kinematics import ReverseKinematics


# Set the background color to white
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')  # Set foreground (text, grid lines) to black


def pg_live_plot_loop(grid, update_interval=10, servo_controller=None):

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

    # Create a container widget to hold the button
    container = QWidget()
    layout = QVBoxLayout()
    container.setLayout(layout)

    # Create a button and add it to the layout
    button = QPushButton("Print to Terminal")
    layout.addWidget(button)

    # Define the button's functionality
    def on_button_click():
        print("Button clicked!")

    button.clicked.connect(on_button_click)

    # Use QGraphicsProxyWidget to embed the QWidget into the PyQtGraph scene
    proxy = QGraphicsProxyWidget()
    proxy.setWidget(container)
    win.scene().addItem(proxy)

    # Position the button in the top-left corner
    proxy.setPos(10, 10)

    # Occupancy Grid Plot (top-left)
    plot_grid = win.addPlot(title="Occupancy Grid")
    plot_grid.setAspectLocked(True)
    gridDiv = grid.grid.shape[0]
    gridSize = gridDiv * grid.cell_size
    plot_grid.setXRange(0, grid.grid.shape[1])
    plot_grid.setYRange(0, grid.grid.shape[0])
    plot_grid.showGrid(x=True, y=True, alpha=0.3)
    img = pg.ImageItem()
    plot_grid.addItem(img)
    robot_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_grid.addItem(robot_dot)
    lut = np.zeros((256, 3), dtype=np.ubyte)
    lut[:128] = [0, 255, 0]    # green for free
    lut[128:] = [255, 0, 0]    # red for occupied

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

    # Add a new plot for robot movement toward the goal (top-right)
    plot_robot_to_goal = win.addPlot(title="Robot Movement to Goal")
    plot_robot_to_goal.setAspectLocked(True)
    plot_robot_to_goal.setXRange(0, grid.grid.shape[1])
    plot_robot_to_goal.setYRange(0, grid.grid.shape[0])
    plot_robot_to_goal.showGrid(x=True, y=True, alpha=0.3)

    # Add the robot's position as a green dot
    robot_position_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 255, 0, 200))  # Green dot
    plot_robot_to_goal.addItem(robot_position_dot)

    # Add the goal position as a red dot
    goal_position_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255, 0, 0, 200))  # Red dot
    plot_robot_to_goal.addItem(goal_position_dot)

    # Add a trajectory line to show the robot's gradual movement
    robot_trajectory_line = pg.PlotDataItem(pen=pg.mkPen('g', width=2))  # Green line
    plot_robot_to_goal.addItem(robot_trajectory_line)

    # Initialize trajectory data
    robot_trajectory_x = []
    robot_trajectory_y = []

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

    # Servo Angles Plot (bottom-right)
    plot_servo = win.addPlot(title="Servo Angles")
    plot_servo.setYRange(0, 180)  # Servo angles range from 0 to 180 degrees
    plot_servo.setXRange(-0.5, 4.5)  # X-axis for 5 servos
    plot_servo.showGrid(x=True, y=True, alpha=0.3)

    # Define colors for each servo
    servo_colors = ['r', 'g', 'b', 'y', 'm']  # Red, Green, Blue, Yellow, Magenta
    servo_bars = pg.BarGraphItem(x=np.arange(5), height=[0]*5, width=0.5, brushes=[pg.mkBrush(color) for color in servo_colors])
    plot_servo.addItem(servo_bars)

    # Add servo names to the X-axis
    servo_names = [f"Servo {i+1}" for i in range(5)]
    axis = plot_servo.getAxis('bottom')
    axis.setTicks([[(i, name) for i, name in enumerate(servo_names)]])

    # Add dynamic labels for degrees at the top of each bar
    degree_labels = []
    for i in range(5):
        label = pg.TextItem(text="", anchor=(0.5, -0.5), color='w')  # Anchor below the text
        plot_servo.addItem(label)
        degree_labels.append(label)

    # Claw Arm Visualization (bottom-center)
    plot_claw = win.addPlot(title="Claw Arm Visualization")
    plot_claw.setAspectLocked(True)
    plot_claw.setXRange(-40, 40)  # Adjust range as needed
    plot_claw.setYRange(-40, 40)  # Adjust range as needed
    plot_claw.showGrid(x=True, y=True, alpha=0.3)

    # Define colors for each servo
    servo_colors = ['g', 'b', 'y']  # Green, Blue, Yellow (for servos 2 to 4)

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


    # Create lines for each arm segment with corresponding colors
    arm_lines = [pg.PlotDataItem(pen=pg.mkPen(color, width=2)) for color in servo_colors]
    for line in arm_lines:
        plot_claw.addItem(line)

    # Update claw color to magenta ('m')
    claw_line_1 = pg.PlotDataItem(pen=pg.mkPen('m', width=2))  # First claw line
    claw_line_2 = pg.PlotDataItem(pen=pg.mkPen('m', width=2))  # Second claw line
    gap_line_1 = pg.PlotDataItem(pen=pg.mkPen('m', width=2))  # Gap (claw extension) for first claw
    gap_line_2 = pg.PlotDataItem(pen=pg.mkPen('m', width=2))  # Gap (claw extension) for second claw
    plot_claw.addItem(claw_line_1)
    plot_claw.addItem(claw_line_2)
    plot_claw.addItem(gap_line_1)
    plot_claw.addItem(gap_line_2)

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

    def update():
        # Occupancy grid
        grid_disp = np.zeros_like(grid.grid, dtype=np.uint8)
        grid_disp[grid.grid <= 0] = 64   # green
        grid_disp[grid.grid > 0] = 192   # red
        img.setImage(np.flipud(grid_disp.T), levels=(0, 255), lut=lut)
        QtCore.QCoreApplication.processEvents()
        img.setRect(QtCore.QRectF(0, 0, grid.grid.shape[1], grid.grid.shape[0]))

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

        robot_dot.setData([robot_ix], [robot_iy])
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

        # Update servo angles
        if servo_controller:

            #servo_controller.current_positions[0] = 45  # Set Servo 1 starting angle to 45 degrees
            servo_angles = servo_controller.current_positions
            servo_bars.setOpts(height=servo_angles)  # Update bar heights with servo angles
            for i, label in enumerate(degree_labels):
                label.setText(f"{servo_angles[i]:.1f}°")  # Update label with angle
                if servo_angles[i] < 20:  # If the bar is small, position the label above the bar
                    label.setAnchor((0.5, 1.5))  # Flip anchor to appear above
                    label.setPos(i, servo_angles[i] + 5)  # Position label above the bar
                else:  # Otherwise, position the label inside the bar
                    label.setAnchor((0.5, -0.5))  # Anchor below the text
                    label.setPos(i, servo_angles[i] - 10)  # Position label inside the bar
        
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


        # Update claw arm visualization
        if servo_controller:
            # Get the adjusted positions for visualization
            visual_positions = servo_controller.get_visual_positions()

            # Get the angles of the arm servos (servo 2 to servo 4)
            angles = [np.radians(visual_positions[i]) for i in range(1, 4)]  # Convert to radians

            # Adjust the 4th servo (3rd arm) so 90 degrees is inline with the previous arm
            angles[2] -= np.pi / 2  # Subtract 90 degrees (π/2 radians)

            # Lengths of the arm segments (in cm)
            lengths = [12, 10, 10]  # Updated lengths for each segment

            # Initialize the base position
            x_positions = [0]
            y_positions = [0]

            # Calculate the positions of the arm segments
            cumulative_angle = 0
            for i in range(3):  # For servos 2 to 4
                cumulative_angle += angles[i]
                x_positions.append(x_positions[-1] + lengths[i] * np.cos(cumulative_angle))
                y_positions.append(y_positions[-1] + lengths[i] * np.sin(cumulative_angle))

            # Update the lines to represent the arm segments
            for i, line in enumerate(arm_lines):
                line.setData([x_positions[i], x_positions[i + 1]], [y_positions[i], y_positions[i + 1]])

            # Calculate and update the claw lines
            claw_base_x = x_positions[-1]
            claw_base_y = y_positions[-1]
            claw_angle = np.radians(servo_controller.current_positions[4])  # Servo 5 angle
            claw_length = 8  # Length of each claw segment
            claw_extension = 2  # Gap (claw extension) in cm

            # Adjust claw angles based on servo 5 position
            # At 0°, the lines touch each other; at 180°, the angle between them is 90°.
            claw_opening_angle = np.radians(servo_controller.current_positions[4]) / 2  # Half of the servo angle

            # Rotate the entire claw 90 degrees clockwise by adding -π/2 to the cumulative angle
            cumulative_angle -= np.pi / 2

            # Calculate the extended base of the claw
            extended_claw_base_x = claw_base_x + claw_extension * np.cos(cumulative_angle)
            extended_claw_base_y = claw_base_y + claw_extension * np.sin(cumulative_angle)

            # Draw the gap (claw extension) as magenta lines
            gap_line_1.setData([claw_base_x, extended_claw_base_x], [claw_base_y, extended_claw_base_y])
            gap_line_2.setData([claw_base_x, extended_claw_base_x], [claw_base_y, extended_claw_base_y])

            # First claw line (rotated counterclockwise)
            claw_1_x = extended_claw_base_x + claw_length * np.cos(cumulative_angle + claw_opening_angle)
            claw_1_y = extended_claw_base_y + claw_length * np.sin(cumulative_angle + claw_opening_angle)
            claw_line_1.setData([extended_claw_base_x, claw_1_x], [extended_claw_base_y, claw_1_y])

            # Second claw line (rotated clockwise)
            claw_2_x = extended_claw_base_x + claw_length * np.cos(cumulative_angle - claw_opening_angle)
            claw_2_y = extended_claw_base_y + claw_length * np.sin(cumulative_angle - claw_opening_angle)
            claw_line_2.setData([extended_claw_base_x, claw_2_x], [extended_claw_base_y, claw_2_y])

            

            

        # Binary map visualization
        binary_map = (grid.grid > 0).astype(int)  # Convert grid to binary map
        binary_img = (binary_map * 255).astype(np.uint8)  # Scale binary values to 0-255
        img_binary.setImage(np.flipud(binary_img.T), levels=(0, 255))
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

        # Update the robot's position and goal on the new plot
        try:
            # Get the robot's current position
            current_x = fused_pose['x']
            current_y = fused_pose['y']

            # Get the goal position
            goal_x, goal_y = grid.goal_x, grid.goal_y

            # Update the robot's position dot
            robot_position_dot.setData([current_x], [current_y])

            # Update the goal position dot
            goal_position_dot.setData([goal_x], [goal_y])

            # Append the robot's position to the trajectory if it has moved
            if not robot_trajectory_x or (abs(current_x - robot_trajectory_x[-1]) > 1e-4 or abs(current_y - robot_trajectory_y[-1]) > 1e-4):
                robot_trajectory_x.append(current_x)
                robot_trajectory_y.append(current_y)

            # Update the trajectory line
            robot_trajectory_line.setData(robot_trajectory_x, robot_trajectory_y)
        except AttributeError:
            # If data is not available yet, clear the plot
            robot_position_dot.setData([], [])
            goal_position_dot.setData([], [])
            robot_trajectory_line.setData([], [])

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(update_interval)

    update()  # Initial draw

    app.exec_()

