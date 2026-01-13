# ===========================================================
# run_all.py — patched minimal for GUI use
# ===========================================================

import sys
import time
from PyQt5.QtWidgets import QApplication

import startup
import testingUART

import encoder_processing
import sonar_processing
import encoder_local_map
import Accel_local_map
import DHT_processing
import obstacle_grid_processing
import reverse_kinematics
import LCD_processing

from claw import start_claw_controller, start_claw_demo_thread

shared_data = None
running_event = None
lcd_proc = None
run_wheels = True


def get_qt_app():
    app = QApplication.instance()
    return app if app else QApplication(sys.argv)


def main():
    global shared_data, running_event, lcd_proc

    print("[run_all] Starting PAUL...")
    get_qt_app()

    shared_data, running_event, ser = startup.startup()
    print("[run_all] Startup complete.")

    encoder_processing.create_and_run(shared_data, poll=0.05)
    sonar_processing.SonarProcessor(shared_data, poll=0.05)
    encoder_local_map.create_and_run(poll=0.05)

    DHT_processing.create_and_run(shared_data, poll=0.1)
    lcd_proc = LCD_processing.create_and_run(shared_data, poll=0.9)

    controller = start_claw_controller(
        shared_data,
        step_sizes=[2,2,3,5,5],
        transition_delay=0.02,
        servo_directions=[1,1,1,1,1]
    )

    start_claw_demo_thread(running_event, controller, whenDetect=0.6)

    if run_wheels:
        Accel_local_map.create_and_run(shared_data)
        obstacle_grid_processing.create_and_run(shared_data)
        reverse_kinematics.create_and_run(shared_data)

    # SLAM GRID INSTANCE
    grid = obstacle_grid_processing.create_and_run(shared_data, poll=0.005)

    return grid, controller
