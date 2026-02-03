# run_all.py
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
import LCD_processing

from claw import start_claw_controller, start_claw_demo_thread
from live_plots import pg_live_plot_loop, QApplication
from PyQt5.QtCore import QCoreApplication

import Payload
import Melodies



# ---------------- GLOBALS ----------------
app = QApplication(sys.argv)

shared_data = None       # <-- Added so SIGINT can see it
running_event = None     # populated in main()
lcd_proc = None

run_wheels = True
# ------------------------------------------


# ---------- SIGINT HANDLER (CTRL-C) ----------
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
    except:
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
    except:
        pass

    print("[Shutdown] Complete.")
    sys.exit(0)



signal.signal(signal.SIGINT, handle_sigint)
# -----------------------------------------------


# =============== MAIN PROGRAM ===================
def main():
    global running_event, shared_data   # <-- Important fix

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

   #Payload.set_motors(shared_data, 20, 20, 20, 20)
    #time.sleep(2.0)

    

    DHT_processing.create_and_run(shared_data, poll=0.1)

    lcd_proc = LCD_processing.create_and_run(shared_data, poll=0.9)

    # Claw control
    controller = start_claw_controller(
        shared_data=shared_data,
        step_sizes=[2, 2, 3, 5, 5],
        transition_delay=0.02,
        servo_directions=[1, 1, 1, 1, 1]
    )
#Justin is a genius
    # Demo claw motion
    start_claw_demo_thread(
        running_event,
        controller,
        whenDetect=0.6
    )

    if run_wheels is True:
        # Accelerometer + IMU maps
        Accel_local_map.create_and_run(shared_data)

        # SLAM grid + obstacles
        obstacle_grid_processing.create_and_run(shared_data)

        # Reverse kinematics node
        reverse_kinematics.create_and_run(shared_data)

    else:
        print("\nSkipping Wheels")
        pass

    # Live pg_plot window
    grid = obstacle_grid_processing.create_and_run(shared_data, poll=0.005)

    print("[System] Launching live plot window...")
    pg_live_plot_loop(grid, servo_controller=controller)
    # ------------------------------------------


    # ---------- MAIN LOOP ----------
    try:
        while running_event.is_set():
            time.sleep(0.1)

    finally:
        print("\n[Finally] Cleanup triggered...")

        # Stop systems
        running_event.clear()

        try:
            testingUART.close_serial()
            print("[Finally] UART closed.")
        except:
            pass

        try:
            app.quit()
            print("[Finally] Qt closed.")
        except:
            pass

        print("[Finally] PAUL shutdown complete.")


# Only run when called directly
if __name__ == "__main__":
    main()
