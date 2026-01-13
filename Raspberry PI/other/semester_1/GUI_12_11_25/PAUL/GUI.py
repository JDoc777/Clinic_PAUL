# ===========================================================
# GUI.py — PATCHED FOR EMBEDDED LIVE PLOTS
# ===========================================================

import sys
import threading
import signal
import time

from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QFrame
from PyQt5.QtCore import QObject, pyqtSignal, QCoreApplication

import run_all
from live_plots import pg_live_plot_loop
import Payload
import Melodies
import testingUART


class PlotSignal(QObject):
    launch = pyqtSignal(object, object)  # grid, controller

plot_signal = PlotSignal()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PAUL Robot GUI")
        self.setGeometry(200, 200, 500, 350)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Select Mode"))

        btn_auto = QPushButton("Autonomous Mode")
        btn_auto.clicked.connect(lambda: self.open_mode("Autonomous Mode", True))
        layout.addWidget(btn_auto)

        btn_measure = QPushButton("Measure Mode")
        btn_measure.clicked.connect(lambda: self.open_mode("Measure Mode", False))
        layout.addWidget(btn_measure)

        btn_control = QPushButton("Control Mode")
        btn_control.clicked.connect(lambda: self.open_mode("Control Mode", None))
        layout.addWidget(btn_control)

        btn_exit = QPushButton("Exit")
        btn_exit.clicked.connect(self.exit_application)
        layout.addWidget(btn_exit)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def open_mode(self, name, wheels):
        self.mode = ModeWindow(name, wheels, parent=self)
        self.mode.show()
        self.hide()

    def exit_application(self):
        handle_sigint(None, None)


class ModeWindow(QMainWindow):
    def __init__(self, mode_name, run_wheels, parent=None):
        super().__init__(parent)

        self.setWindowTitle(mode_name)
        self.setGeometry(200, 200, 1200, 900)
        self.run_wheels = run_wheels
        self.worker = None
        self.plot_widget = None

        self.layout = QVBoxLayout()
        self.layout.addWidget(QLabel(f"{mode_name} — Wheel Systems: {run_wheels}"))

        btn_start = QPushButton("Start System")
        btn_start.clicked.connect(self.start_mode)
        self.layout.addWidget(btn_start)

        btn_stop = QPushButton("Stop System")
        btn_stop.clicked.connect(self.stop_mode)
        self.layout.addWidget(btn_stop)

        btn_back = QPushButton("Back to Main Menu")
        btn_back.clicked.connect(self.go_back)
        self.layout.addWidget(btn_back)

        # FRAME FOR PLOT
        self.plot_frame = QFrame()
        self.plot_frame.setFrameShape(QFrame.StyledPanel)
        self.plot_frame.setMinimumHeight(700)
        self.plot_layout = QVBoxLayout()
        self.plot_frame.setLayout(self.plot_layout)
        self.layout.addWidget(self.plot_frame)

        container = QWidget()
        container.setLayout(self.layout)
        self.setCentralWidget(container)

        plot_signal.launch.connect(self.embed_plot)

    def start_mode(self):
        print("\nGUI: Starting mode thread...")
        run_all.run_wheels = self.run_wheels

        if not self.worker or not self.worker.is_alive():
            self.worker = threading.Thread(target=self.worker_task, daemon=True)
            self.worker.start()

    def worker_task(self):
        try:
            grid, controller = run_all.main()
            print("GUI: run_all returned → embedding plot")
            plot_signal.launch.emit(grid, controller)
        except Exception as e:
            print("ERROR in worker thread:", e)

    def stop_mode(self):
        handle_sigint(None, None)

    def embed_plot(self, grid, controller):
        print("GUI: Embedding plot widget...")

        if self.plot_widget:
            self.plot_layout.removeWidget(self.plot_widget)
            self.plot_widget.deleteLater()

        self.plot_widget = pg_live_plot_loop(
            grid,
            update_interval=10,
            servo_controller=controller,
            embedded=True,
        )

        self.plot_layout.addWidget(self.plot_widget)
        self.plot_widget.show()
        print("GUI: Plot embedded successfully.")

    def go_back(self):
        if self.parent():
            self.parent().show()
        self.close()


def handle_sigint(signum, frame):
    print("Shutdown triggered...")
    QCoreApplication.quit()
    time.sleep(0.5)

    try:
        Payload.set_motors(run_all.shared_data, 0, 0, 0, 0)
        testingUART.close_serial()
    except:
        pass

    sys.exit(0)


if __name__ == "__main__":
    print("GUI: Launching Application...")
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()

    signal.signal(signal.SIGINT, handle_sigint)
    sys.exit(app.exec_())
