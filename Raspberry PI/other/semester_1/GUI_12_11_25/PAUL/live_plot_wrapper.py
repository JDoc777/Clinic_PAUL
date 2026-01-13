# live_plot_wrapper.py
"""
Non-blocking plot wrapper for PAUL GUI.
Ensures plot windows do not freeze or close the GUI.
"""

from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt

from live_plots import pg_live_plot_loop


class PlotWindow(QMainWindow):
    def __init__(self, grid, controller):
        super().__init__()
        self.setWindowTitle("PAUL Live Plot")
        self.setGeometry(350, 200, 800, 600)

        container = QWidget()
        layout = QVBoxLayout()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Embed plot widget
        pg_live_plot_loop(grid, controller, embed_in=layout)


def open_live_plot_window(grid, controller):
    win = PlotWindow(grid, controller)
    win.show()