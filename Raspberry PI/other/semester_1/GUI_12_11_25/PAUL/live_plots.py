# ===========================================================
# live_plots.py — PATCHED FOR EMBEDDED GUI MODE
# ===========================================================

import sys
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QApplication

from obstacle_grid_processing import fused_pose

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


def pg_live_plot_loop(grid, update_interval=10, servo_controller=None, embedded=False):
    """
    Returns a GraphicsLayoutWidget that updates continuously when embedded=True.
    """

    # -----------------------------------------------------------
    # USE EXISTING QT APPLICATION
    # -----------------------------------------------------------
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QApplication([])

    # -----------------------------------------------------------
    # MAIN WIDGET (this is what gets embedded into the GUI)
    # -----------------------------------------------------------
    win = pg.GraphicsLayoutWidget(show=not embedded)
    win.resize(1800, 800)

    # ------------------------------
    # OCCUPANCY GRID SETUP
    # ------------------------------
    plot_grid = win.addPlot(title="Occupancy Grid")
    plot_grid.setAspectLocked(True)
    plot_grid.setXRange(0, grid.grid.shape[1])
    plot_grid.setYRange(0, grid.grid.shape[0])
    plot_grid.showGrid(x=True, y=True, alpha=0.3)

    img = pg.ImageItem()
    plot_grid.addItem(img)

    robot_dot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 0, 255, 200))
    plot_grid.addItem(robot_dot)

    # LUT for free/occupied cells
    lut = np.zeros((256, 3), dtype=np.uint8)
    lut[:128] = [0, 255, 0]
    lut[128:] = [255, 0, 0]

    # ===========================================================
    # UPDATE LOOP
    # ===========================================================
    def update():
        # -------------------------------------------
        # OCCUPANCY GRID
        # -------------------------------------------
        grid_disp = np.zeros_like(grid.grid, dtype=np.uint8)
        grid_disp[grid.grid <= 0] = 64
        grid_disp[grid.grid > 0] = 192
        img.setImage(np.flipud(grid_disp.T), levels=(0, 255), lut=lut)

        # -------------------------------------------
        # ROBOT POSITION
        # -------------------------------------------
        x_pos = fused_pose['x']
        y_pos = fused_pose['y']

        # convert robot world position to grid index
        ix = int((x_pos - grid.x_origin) / grid.cell_size)
        iy = int((y_pos - grid.y_origin) / grid.cell_size)

        robot_dot.setData([ix], [iy])

    # -----------------------------------------------------------
    # TIMER — MUST BE STORED TO PREVENT GC
    # -----------------------------------------------------------
    win._timer = QtCore.QTimer()
    win._timer.timeout.connect(update)
    win._timer.start(update_interval)

    update()  # first frame

    # -----------------------------------------------------------
    # RETURN EMBEDDED WIDGET
    # -----------------------------------------------------------
    if embedded:
        print("live_plots: Returning embedded plot widget.")
        return win

    # Otherwise standalone mode
    app.exec_()
    return win
