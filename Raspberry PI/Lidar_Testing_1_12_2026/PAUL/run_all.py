import sys
import time
import signal

import startup
import testingUART

import encoder_processing
import sonar_processing
import encoder_local_map
import Accel_local_map
import DHT_processing
import obstacle_grid_processing
import reverse_kinematics
import local_motion_testing
import LCD_processing
import lidar_processing
import lidar_local_map
import lidar_obstacle_map

from claw import start_claw_controller, start_claw_demo_thread
from live_plots import pg_live_plot_loop, QApplication
from PyQt5.QtCore import QCoreApplication
from PyQt5 import QtWidgets

import Payload
import Melodies


# ---------------- GLOBALS ----------------
app = QApplication(sys.argv)

shared_data = None       # <-- Added so SIGINT can see it
running_event = None     # populated in main()
lcd_proc = None
controller = None        # claw controller (for demo thread)
run_wheels = False       # autonomous movement flag
toggle_window = None     # control panel window
claw_demo_thread = None  # handle to claw demo thread
# ------------------------------------------

class ManualControlPad(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PAUL Manual Drive Pad")
        self.setGeometry(400, 100, 300, 300)

        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)

        # Helper for making buttons
        def make_button(label, row, col, press_fn, release_fn):
            btn = QtWidgets.QPushButton(label)
            btn.setFixedSize(80, 80)
            btn.setStyleSheet("font-size: 16px;")
            btn.pressed.connect(press_fn)
            btn.released.connect(release_fn)
            layout.addWidget(btn, row, col)
            return btn

        # ---- BUTTON CALLBACKS ----
        def stop():
            Payload.set_motors(shared_data, 0, 0, 0, 0)

        def forward():
            Payload.set_motors(shared_data, 50, 50, 50, 50)

        def backward():
            Payload.set_motors(shared_data, -50, -50, -50, -50)

        def crab_left():
            Payload.set_motors(shared_data, -50, 50, 50, -50)

        def crab_right():
            Payload.set_motors(shared_data, 50, -50, -50, 50)

        def diag_forward_left():
            Payload.set_motors(shared_data, 0, 50, 50, 0)

        def diag_forward_right():
            Payload.set_motors(shared_data, 50, 0, 0, 50)

        def diag_back_left():
            Payload.set_motors(shared_data, 0, -50, -50, 0)

        def diag_back_right():
            Payload.set_motors(shared_data, -50, 0, 0, -50)

        def rotate_ccw():
            Payload.set_motors(shared_data, -50, 50, -50, 50)

        def rotate_cw():
            Payload.set_motors(shared_data, 50, -50, 50, -50)

        # 3×3 grid layout:
        #
        #   ↖    ↑     ↗
        #   ←  ROT   →
        #   ↙    ↓     ↘
        #

        make_button("↖", 0, 0, diag_forward_left, stop)
        make_button("↑", 0, 1, forward, stop)
        make_button("↗", 0, 2, diag_forward_right, stop)

        make_button("←", 1, 0, crab_left, stop)

        # CENTER SPLIT INTO TWO ROTATE BUTTONS
        rotate_widget = QtWidgets.QWidget()
        rotate_layout = QtWidgets.QHBoxLayout()
        rotate_widget.setLayout(rotate_layout)
        btn_ccw = QtWidgets.QPushButton("⟲")
        btn_cw = QtWidgets.QPushButton("⟳")

        btn_ccw.setFixedSize(40, 80)
        btn_cw.setFixedSize(40, 80)

        btn_ccw.pressed.connect(rotate_ccw)
        btn_ccw.released.connect(stop)

        btn_cw.pressed.connect(rotate_cw)
        btn_cw.released.connect(stop)

        rotate_layout.addWidget(btn_ccw)
        rotate_layout.addWidget(btn_cw)

        layout.addWidget(rotate_widget, 1, 1)

        make_button("→", 1, 2, crab_right, stop)

        make_button("↙", 2, 0, diag_back_left, stop)
        make_button("↓", 2, 1, backward, stop)
        make_button("↘", 2, 2, diag_back_right, stop)


class ToggleWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PAUL Control Panel")
        self.setGeometry(100, 100, 260, 200)

        self.layout = QtWidgets.QVBoxLayout()
        self.setLayout(self.layout)

        # -------------------------------
        # STATE VARIABLES
        # -------------------------------
        self.debug_flag = False
        self.auto_flag = False
        self.claw_flag = False

        # -------------------------------
        # BUTTON 1 — DEBUG
        # -------------------------------
        self.btn_debug = QtWidgets.QPushButton("Debug Mode OFF")
        self.btn_debug.setStyleSheet("font-size: 14px;")
        self.btn_debug.clicked.connect(self.toggle_debug)
        self.layout.addWidget(self.btn_debug)

        # -------------------------------
        # BUTTON 2 — AUTONOMOUS MOVEMENT
        # -------------------------------
        self.btn_auto = QtWidgets.QPushButton("Autonomous Movement OFF")
        self.btn_auto.setStyleSheet("font-size: 14px;")
        self.btn_auto.clicked.connect(self.toggle_auto)
        self.layout.addWidget(self.btn_auto)

        # -------------------------------
        # BUTTON 3 — CLAW ARM
        # -------------------------------
        self.btn_claw = QtWidgets.QPushButton("Claw Arm OFF")
        self.btn_claw.setStyleSheet("font-size: 14px;")
        self.btn_claw.clicked.connect(self.toggle_claw)
        self.layout.addWidget(self.btn_claw)

    # ----------------------------------
    #  TOGGLE FUNCTIONS
    # ----------------------------------
    def toggle_debug(self):
        self.debug_flag = not self.debug_flag
        print("Debug Mode =", self.debug_flag)

        if self.debug_flag:
            self.btn_debug.setText("Debug Mode ON")
        else:
            self.btn_debug.setText("Debug Mode OFF")

    def toggle_auto(self):
        """
        Toggle autonomous movement.
        - Updates self.auto_flag
        - Updates global run_wheels
        """
        global run_wheels

        self.auto_flag = not self.auto_flag
        run_wheels = self.auto_flag

        print("Autonomous Movement =", self.auto_flag)

        if self.auto_flag:
            self.btn_auto.setText("Autonomous Movement ON")
            print("[Auto] run_wheels set to True")
        else:
            self.btn_auto.setText("Autonomous Movement OFF")
            print("[Auto] run_wheels set to False")
            # Safe behavior for now: just stop motors
            try:
                if shared_data is not None:
                    Payload.set_motors(shared_data, 0, 0, 0, 0)
                    print("[Auto] Motors commanded to stop.")
            except Exception as e:
                print("[Auto] Error stopping motors:", e)

    def toggle_claw(self):
        """
        Toggle claw demo behavior.
        - When turning ON: start demo thread (if not already running)
        - When turning OFF: just print for now (stopping cleanly would require support in claw module)
        """
        global claw_demo_thread, running_event, controller

        self.claw_flag = not self.claw_flag
        print("Claw Arm =", self.claw_flag)

        if self.claw_flag:
            self.btn_claw.setText("Claw Arm ON")

            if controller is None:
                print("[Claw] WARNING: controller not initialized yet.")
                return

            if (claw_demo_thread is None) or (not claw_demo_thread.is_alive()):
                try:
                    print("[Claw] Starting claw demo thread...")
                    claw_demo_thread = start_claw_demo_thread(
                        running_event,
                        controller,
                        whenDetect=0.6
                    )
                    print("[Claw] Demo thread started.")
                except Exception as e:
                    print("[Claw] ERROR starting demo thread:", e)
            else:
                print("[Claw] Demo thread already running.")
        else:
            self.btn_claw.setText("Claw Arm OFF")
            # NOTE: We don't have a clean stop hook for the thread yet.
            # For now, just print. To actually stop, the claw thread code
            # should check a shared flag and exit its loop gracefully.
            print("[Claw] Claw OFF requested (thread will continue until stop logic is added).")


# ---------- SIGINT HANDLER (CTRL-C) ----------
def handle_sigint(signum, frame):
    print("KeyboardInterrupt (SIGINT) received, preparing shutdown...")
    QCoreApplication.quit()
    time.sleep(1.0)

    try:
        # ---------------------------------------------------------
        # 1. STOP MOTORS
        # ---------------------------------------------------------
        Payload.set_motors(shared_data, 0, 0, 0, 0)
        print("[Shutdown] Motors stopped.")
        QCoreApplication.quit()  # Gracefully quit the Qt event loop
        time.sleep(1.0)

        # ---------------------------------------------------------
        # 2. STOP LCD THREAD BEFORE ANY LCD COMMANDS
        # ---------------------------------------------------------
        try:
            if lcd_proc is not None:
                lcd_proc.stop()
                print("[Shutdown] LCD thread stopped.")
        except Exception as e:
            print("[Shutdown] Failed to stop LCD thread:", e)

        # ---------------------------------------------------------
        # 3. SEND SHUTDOWN LCD MESSAGE
        # ---------------------------------------------------------
        Payload.set_text(shared_data, "PAUL SHUTTING DOWN...")
        print("[Shutdown] LCD message displayed.")
        time.sleep(1.5)

        # ---------------------------------------------------------
        # 4. PLAY SHUTDOWN MELODY
        # ---------------------------------------------------------
        Melodies.play("power_down")
        print("[Shutdown] Shutdown melody triggered.")
        time.sleep(0.5)

        # ---------------------------------------------------------
        # 5. SEND FINAL FLAG (RESET + LED)
        # ---------------------------------------------------------
        Payload.set_flags_byte(shared_data, 0b00111001)
        print("[Shutdown] Final flags sent.")
        time.sleep(1.0)

    except Exception as e:
        print("[Shutdown] ERROR during shutdown:", e)

    # ---------------------------------------------------------
    # 6. QUIT QT LOOP
    # ---------------------------------------------------------
    try:
        app.quit()
        print("[Shutdown] Qt terminated.")
    except Exception:
        pass

    # ---------------------------------------------------------
    # 7. STOP OTHER THREADS
    # ---------------------------------------------------------
    if running_event:
        running_event.clear()

    # ---------------------------------------------------------
    # 8. CLOSE UART LAST
    # ---------------------------------------------------------
    try:
        testingUART.close_serial()
    except Exception:
        pass

    print("[Shutdown] Complete.")
    sys.exit(0)


signal.signal(signal.SIGINT, handle_sigint)
# -----------------------------------------------


# =============== MAIN PROGRAM ===================
def main():
    global running_event, shared_data, toggle_window, controller, lcd_proc

    # ---------- STARTUP SEQUENCE ----------
    shared_data, running_event, ser = startup.startup()
    print("[Startup] PAUL startup complete.")
    # --------------------------------------

    # ---------- Start all subsystems ----------
    processor = encoder_processing.create_and_run(shared_data, poll=0.05)

    sonar_proc = sonar_processing.SonarProcessor(shared_data, poll=0.05)

    enc_local_map = encoder_local_map.create_and_run(
        poll=0.05,
        debug=True
    )

    


    DHT_processing.create_and_run(shared_data, poll=0.1)

    lcd_proc = LCD_processing.create_and_run(shared_data, poll=0.9)

    # Claw controller
    controller = start_claw_controller(
        shared_data=shared_data,
        step_sizes=[2, 2, 3, 5, 5],
        transition_delay=0.02,
        servo_directions=[1, 1, 1, 1, 1]
    )

    # NOTE: We no longer auto-start the claw demo thread here.
    # It will be controlled by the "Claw Arm" toggle button instead.


    # Wheels / autonomous subsystems at startup
    if run_wheels is True:
        
        # Accelerometer + IMU maps
        Accel_local_map.create_and_run(shared_data)

        # SLAM grid + obstacles
        obstacle_grid_processing.create_and_run(shared_data)

        # Reverse kinematics node
        local_motion_testing.create_and_run(shared_data)

    else:
        print("\n[Startup] Skipping Wheels (run_wheels is False at startup)")

    # Live pg_plot window
    grid = obstacle_grid_processing.create_and_run(shared_data, poll=0.005)

    lidar_proc = lidar_processing.LidarProcessor(shared_data, debug=True)

    lidar_map = lidar_local_map.LidarLocalMap(lidar_proc, poll=0.02)

    # 🔥 IMPORTANT: pass lidar_proc, NOT lidar_map
    lidar_obs = lidar_obstacle_map.LidarObstacleMap(lidar_proc, grid, poll=0.02)

    grid.lidar_map = lidar_map
    grid.lidar_obs = lidar_obs

    #local_motion_testing.create_and_run(shared_data, poll=0.05)




    






    print("[System] Launching live plot window...")

    # ----- NEW CONTROL WINDOW -----
    toggle_window = ToggleWindow()
    toggle_window.show()

    manual_pad = ManualControlPad()
    manual_pad.show()


    # ----- MAIN LIVE PLOT -----
    pg_live_plot_loop(grid, servo_controller=controller)
    # ------------------------------------------

    # ---------- MAIN GOON LOOP ----------
    try:
        wheel_system_started = False

        while running_event.is_set():

            # ================================
            #   AUTONOMOUS MOVEMENT CONTROL
            # ================================
            if toggle_window:

                # --- ENABLE WHEELS ---
                if toggle_window.auto_flag and not wheel_system_started:
                    print("[AUTO] Starting wheel subsystems...")

                    Accel_local_map.create_and_run(shared_data)
                    obstacle_grid_processing.create_and_run(shared_data)

                    wheel_system_started = True

                # --- DISABLE WHEELS ---
                if not toggle_window.auto_flag and wheel_system_started:
                    print("[AUTO] Stopping wheel motion...")

                    Payload.set_motors(shared_data, 0, 0, 0, 0)
                    wheel_system_started = False
            
            time.sleep(0.1)

    finally:
        print("\n[Finally] Cleanup triggered...")

        # Stop systems
        running_event.clear()

        try:
            testingUART.close_serial()
            print("[Finally] UART closed.")
        except Exception:
            pass

        try:
            app.quit()
            print("[Finally] Qt closed.")
        except Exception:
            pass

        print("[Finally] PAUL shutdown complete.")


# Only run when called directly
if __name__ == "__main__":
    main()
