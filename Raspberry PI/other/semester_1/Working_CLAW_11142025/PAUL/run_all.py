# single-process orchestrator: start uart reader then start encoder processor (no double serial open)
import testingUART
import encoder_processing
import sonar_processing
import Payload
import encoder_local_map
import gyro_plot
import time
import matplotlib.pyplot as plt  # added to allow plt.pause() and plt.close()
import matplotlib.animation as ani
import PIDmotor
import sonar_obstacle_local_map
import obstacle_grid_processing
import Accel_local_map
import threading
from gyro_plot import plot_gyro_accel
from obstacle_grid_processing import ObstacleGrid
from live_plots import pg_live_plot_loop, QApplication
import sys
import signal
from claw import ServoController, start_claw_controller, start_claw_demo_thread

# ------------------- CONFIG / TUNABLES -------------------
ENABLE_GRAPHS = False
CLAW_STEP_SIZES = [2, 2, 3, 5, 5]  # degrees per step for each servo
CLAW_TRANSITION_DELAY = 0.02
CLAW_DEMO_whenDetect = 0.6
# ---------------------------------------------------------

# Servo configuration
SERVO_DIRECTIONS = [1, -1, 1, -1, 1]  # Example: reverse servos 2 and 4


app = QApplication(sys.argv)

def handle_sigint(signum, frame):
    print("KeyboardInterrupt (SIGINT) received, quitting Qt event loop...")
    app.quit()  # This will cause app.exec_() to return
    Payload.set_flags_byte(shared_data, 0b00111001)  # example: set flags directly (LED1 + RESET)
    sys.exit(0)


signal.signal(signal.SIGINT, handle_sigint)


def main():   
    Rst = 0

    # start the UART reader in background; it owns the serial port
    global shared_data, running_event, ser
    shared_data, running_event, ser = testingUART.start_reader(start_writer=True, do_handshake=True)


    processor = encoder_processing.create_and_run(shared_data, poll=0.05)
    #motor = PIDmotor.PIDMotor(processor)  # Initialize PIDMotor with encoder processor

    # Set PID constants directly
    #motor.set_pid_constants(kp=3, ki=1, kd=1)


    # Example: Set wheel speeds in meters per second
    wheel_speeds = {'set_FR': 2, 'set_FL': 2, 'set_RR': 2, 'set_RL': 2}  # Example speeds
    #motor.set_wheel_speeds(wheel_speeds)



    # Start a single sonar processor using the shared_data from UART
    sonar_proc = sonar_processing.SonarProcessor(shared_data, poll=0.05)

    # Pass that same processor to the local map – do NOT spawn another processor
    #sonar_local_map = sonar_obstacle_local_map.SonarObstacleLocalMap(sonar_processor=sonar_proc, shared_data=shared_data, poll=0.05)


    #sonar_proc = sonar_processing.create_and_run(shared_data, poll=0.05)
    #sonar_local_map = sonar_obstacle_local_map.SonarObstacleLocalMap(sonar_proc)
    #sonar_local_map = sonar_local_map.create_and_run(shared_data=shared_data, poll=0.05)

    enc_local_map = encoder_local_map.create_and_run(poll=0.05, debug=True)
    

    #Payload.set_text(shared_data, "Starting in 5 seconds")  # example: send text to Arduino
    #Payload.set_text(shared_data, "ITs NO GO TIME  :)")  # example: send text to Arduino
    #Payload.set_flags_byte(shared_data, 0b00010111)


    # start local map updater that uses the same shared_data


    # Start Accel_local_map in its own thread so it runs concurrently
    
    Payload.set_text(shared_data, "Starting in 3 seconds")  # example: send text to Arduino
    Payload.set_text(shared_data, "ITS GO TIME :)")  # example: send text to Arduino
    Payload.set_flags_byte(shared_data, 0b00101001)  # example: set flags directly (LED1 + RESET)
    
    
    # ---- CLAW: start controller and demo using helpers ----
    controller = start_claw_controller(
        step_sizes=CLAW_STEP_SIZES,
        transition_delay=CLAW_TRANSITION_DELAY,
        shared_data=shared_data,
        servo_directions=SERVO_DIRECTIONS
    )

    # demo thread (optional)
    start_claw_demo_thread(running_event, controller, whenDetect=CLAW_DEMO_whenDetect)

    # simple manual test sequence (replace with demo thread if desired)
    # threading.Thread(target=controller.start_non_blocking_sequence, daemon=True).start()
    # ---- END CLAW ----
    
    Accel_local_map.create_and_run(shared_data)
    obstacle_grid_processing.create_and_run(shared_data)
    grid = obstacle_grid_processing.create_and_run(shared_data)
    pg_live_plot_loop(grid, servo_controller=controller)
    
    
    #Payload.buzzer(shared_data, 2,`` 90)  # example: set buzzer frequency and duration
    #Payload.set_motors(shared_data, fl=200, fr=1, rl=200, rr=1)  # example: set all motors forward at 200
    #Payload.happy_birthday(shared_data, tempo=0.4, blocking=False, repeat=1)
    # Example: play a single buzzer tone (e.g., 3520 Hz)
    # Example: play happy birthday melody (non-blocking)
    #Payload.happy_birthday(shared_data, tempo=0.4, blocking=False, repeat=1)
    #time.sleep(5)

    #Payload.set_servos(shared_data, 90, 90, 90, 90, 90)  # example: set all servos to neutral (90 degrees)
    #time.sleep(5)
    # Flags byte layout (bits MSB->LSB: b7 b6 b5 b4 b3 b2 b1 b0)
    # b7:RESERVED b6:RESERVED b5:Buzzer b4:RESET b3:handshake b2:MOTOR_ENABLE b1:led1 b0:led0

    #Payload.set_servos(shared_data, 180, 180, 180, 180, 180)  # example: set both servos to neutral (90 degrees)
    #time.sleep(5)

    #Payload.set_servos(shared_data, 0, 0, 0, 0, 0)  # example: set both servos to neutral (90 degrees)
    #time.sleep(5)

    #Payload.set_servos(shared_data, 45, 45, 45, 45, 45)
    
    #Payload.set_servos(shared_data, 90, 90, 90, 90, 90)  # example: set all servos to neutral (90 degrees)

    #time.sleep(2)

    #Payload.set_servos(shared_data, 45, 45, 45, 45, 45)  # example: set all servos to neutral (90 degrees)

    #time.sleep(2)

    #Payload.set_servos(shared_data, 65, 65, 65, 65, 65)  # example: set all servos to neutral (90 degrees)

    # ensure fig and ani exist even if the live-plot line is commented out 
    fig = None
    ani = None

    enable_graphs = False  # Toggle to enable or disable graphs

    if enable_graphs:
        # Initialize live plot only if graphs are enabled
        fig, ani = encoder_local_map.start_live_plot(enc_local_map, interval_ms=1, history_len=5000)
        pass

    # Start the gyro and accel plotting in a separate thread
    #plot_thread = threading.Thread(target=plot_gyro_accel, daemon=True)
    #plot_thread.start()

    try:
        # Use plt.pause when a figure exists so GUI events are processed.
        # This keeps the animatbv ion responsive and prevents the white-box-on-exit symptom.
        while running_event.is_set():
            if enable_graphs and fig and ani is not None:
                # Process GUI events only if graphs are enabled
                plt.pause(0.1)
            else:
                time.sleep(0.1)

            # Display velocities in the loop
            #motor.display_velocities()
            #motor.calculate_velocity_error(wheel_speeds)
            #pwm_values = motor.calculate_pwm(wheel_speeds)
            #print(f"PWM Values: {pwm_values}")
            #Payload.set_motors(shared_data, fl=pwm_values['PWM_FL'], fr=pwm_values['PWM_FR'], rl=pwm_values['PWM_RL'], rr=pwm_values['PWM_RR'])
    finally:
        
# ... later, when you want to stop:
        # stop everything
        time.sleep(1)
        #grid.stop()
        #grid.plot_grid()  # <-- This will show the grid
        #Payload.set_motors(shared_data, 1, 1, 1, 1)  # stop motors
        #processor.stop()
        #sonar_proc.stop()
        #enc_local_map.stop()
        running_event.clear()
        #plot_thread.join()  # Ensure the plot thread stops cleanly
        testingUART.close_serial()


        if enable_graphs and fig and ani is not None:
            # Close the figure cleanly only if graphs are enabled
            try:
                plt.close(fig)
            except Exception:
                pass

if __name__ == "__main__":
    main()