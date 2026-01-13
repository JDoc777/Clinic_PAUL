from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QMessageBox
from PyQt5.QtCore import Qt
import subprocess
import signal
import os  # Add this import at the top to fix the undefined 'os' errors

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Interactive GUI")
        self.setGeometry(100, 100, 400, 300)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Buttons for main menu
        self.autonomous_button = QPushButton("Autonomous Mode")
        self.map_button = QPushButton("Map Mode")
        self.control_button = QPushButton("Control Mode")
        self.exit_button = QPushButton("Exit")

        # Connect buttons to their respective functions
        self.autonomous_button.clicked.connect(self.open_autonomous_mode)
        self.map_button.clicked.connect(self.open_map_mode)
        self.control_button.clicked.connect(self.open_control_mode)
        self.exit_button.clicked.connect(self.close_application)

        # Add buttons to layout
        layout.addWidget(self.autonomous_button)
        layout.addWidget(self.map_button)
        layout.addWidget(self.control_button)
        layout.addWidget(self.exit_button)

        # Set central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def open_autonomous_mode(self):
        self.mode_window = ModeWindow("Autonomous Mode", run_command="/home/PAUL/Documents/GitHub/Clinic_PAUL/'Raspberry PI'/Reverse_KInematics_w_GUI_12_11_2025/PAUL/run_all.py", end_command=signal.SIGINT)
        self.mode_window.show()

    def open_map_mode(self):
        self.mode_window = ModeWindow("Map Mode", run_command="/home/PAUL/Documents/GitHub/Clinic_PAUL/'Raspberry PI'/Reverse_KInematics_w_GUI_12_11_2025/PAUL/run_all.py --run_wheels=False", end_command=signal.SIGINT)
        self.mode_window.show()

    def open_control_mode(self):
        self.mode_window = ModeWindow("Control Mode", run_command=None, end_command=None)
        self.mode_window.show()

    def close_application(self):
        reply = QMessageBox.question(self, 'Exit', 'Are you sure you want to exit?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            QApplication.quit()

class ModeWindow(QMainWindow):
    def __init__(self, mode_name, run_command=None, end_command=None):
        super().__init__()
        self.setWindowTitle(mode_name)
        self.setGeometry(150, 150, 400, 300)

        self.run_command = run_command
        self.end_command = end_command
        self.process = None

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Buttons for mode controls
        self.start_button = QPushButton("Start")
        self.end_button = QPushButton("End")
        self.back_button = QPushButton("Go Back")
        self.back_button.setEnabled(False)  # Initially disabled

        # Connect buttons to their respective functions
        self.start_button.clicked.connect(self.start_mode)
        self.end_button.clicked.connect(self.end_mode)
        self.back_button.clicked.connect(self.go_back)

        # Add buttons to layout
        layout.addWidget(self.start_button)
        layout.addWidget(self.end_button)
        layout.addWidget(self.back_button)

        # Set central widget
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def start_mode(self):
        if self.run_command:
            self.process = subprocess.Popen(self.run_command, shell=True, preexec_fn=os.setsid)

    def end_mode(self):
        if self.process:
            os.killpg(os.getpgid(self.process.pid), self.end_command)
            self.process = None
        self.back_button.setEnabled(True)

    def go_back(self):
        self.close()

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)

    # Open all graphs after startup
    subprocess.Popen("/home/PAUL/Documents/GitHub/Clinic_PAUL/'Raspberry PI'/Reverse_KInematics_w_GUI_12_11_2025/PAUL/live_plots.py", shell=True)
    subprocess.Popen("/home/PAUL/Documents/GitHub/Clinic_PAUL/'Raspberry PI'/Reverse_KInematics_w_GUI_12_11_2025/PAUL/gyro_plot.py", shell=True)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())