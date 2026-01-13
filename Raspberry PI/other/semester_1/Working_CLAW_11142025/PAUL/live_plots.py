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

def pg_live_plot_loop(grid, update_interval=20, servo_controller=None):
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

    # Occupancy Grid Plot (top-left)
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

    # Log-Odds Grid Plot (top-right)
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

    # Corrected Positions Plot (top-right)
    plot_corrected = win.addPlot(title="Corrected Positions (Fused)")
    plot_corrected.setAspectLocked(True)
    plot_corrected.setXRange(-2, 2)
    plot_corrected.setYRange(-2, 2)
    plot_corrected.showGrid(x=True, y=True, alpha=0.3)
    img_corrected = pg.ImageItem()
    plot_corrected.addItem(img_corrected)

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

    win.nextRow()  # Move to the next row for the bottom plots

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

        # Corrected positions visualization
        try:
            # Ensure corrected_grid is properly normalized and visualized
            corrected_disp = np.clip(grid.grid, -10, 10)  # Use the same grid data as log-odds for now
            # Normalize to 0-255 for display: -10 -> 0, 0 -> 127, +10 -> 255
            corrected_img = ((corrected_disp + 10) * (255.0 / 20)).astype(np.uint8)
            img_corrected.setImage(np.flipud(corrected_img.T), levels=(0, 255))
            img_corrected.setRect(QtCore.QRectF(-2, -2, 4, 4))
        except AttributeError:
            # If corrected_grid is not available, clear the image
            img_corrected.clear()

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

        # Update servo angles
        if servo_controller:
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

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(update_interval)


    update()  # Initial draw
    


    app.exec_()
