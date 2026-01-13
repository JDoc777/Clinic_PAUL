# single-process orchestrator: start uart reader then start encoder processor (no double serial open)
import testingUART
import encoder_processing
import sonar_processing
import Payload
import encoder_local_map
import time
import matplotlib.pyplot as plt  # added to allow plt.pause() and plt.close()
import matplotlib.animation as ani
import PIDmotor
import gui



def main():

    Rst = 0

    # start the UART reader in background; it owns the serial port
    shared_data, running_event, ser = testingUART.start_reader(start_writer=True, do_handshake=True)

    if Rst==0:
        Payload.set_flags_byte(shared_data, 0b00010000)  # example: set flags directly (LED1 + RESET)
        Rst = 1
        
    else:
        pass
    

    processor = encoder_processing.create_and_run(shared_data, poll=0.05)
    motor = PIDmotor.PIDMotor(processor)  # Initialize PIDMotor with encoder processor

    # Set PID constants directly
    motor.set_pid_constants(kp=0.5, ki=0.5, kd=0.5)


    # Example: Set wheel speeds in meters per second
    wheel_speeds = {'set_FR': 1, 'set_FL': 1, 'set_RR': 1, 'set_RL': 1}  # Example speeds
    motor.set_wheel_speeds(wheel_speeds)

    sonar_proc = sonar_processing.create_and_run(shared_data, poll=0.05)


    enc_local_map = encoder_local_map.create_and_run(poll=0.05, debug=True)




    # start local map updater that uses the same shared_data



    Payload.set_text(shared_data, "Starting in 5 seconds")  # example: send text to Arduino
    Payload.set_text(shared_data, "ITS GO TIME")  # example: send text to Arduino
    Payload.set_flags_byte(shared_data, 0b00000100)  # example: set flags directly (LED1 + RESET)
    # Flags byte layout (bits MSB->LSB: b7 b6 b5 b4 b3 b2 b1 b0)
    # b7:RESERVED b6:RESERVED b5:LED1 b4:RESET b3:LED0 b2:MOTOR_ENABLE b1:SENSOR_TARE b0:HEARTBEAT

    #Payload.set_motors(shared_data, fl=1, fr=1, rl=1, rr=1)  # example: set all motors forward at 100
    #Payload.set_servos(shared_data, 90, 90, 45, 135)  # example: set both servos to neutral (90 degrees)
   
    # ensure fig and ani exist even if the live-plot line is commented out
    fig = None
    ani = None

    # call the live-plot initializer (can be commented out safely now)
    #fig, ani = encoder_local_map.start_live_plot(enc_local_map, interval_ms=1, history_len=5000)

    try:
        # Start Tkinter GUI which will poll shared_data and other components.
        # This call blocks until the GUI window is closed. Pass `motor` so
        # the GUI can request calculated PWMs (e.g., set wheels to 1 m/s).
        # After GUI exits, main loop will continue and proceed to shutdown.
        while running_event.is_set():
            # keep a short sleep so we can break out if running_event cleared elsewhere
            time.sleep(0.1)

            # Display velocities in the loop
            motor.display_velocities()
            motor.calculate_velocity_error(wheel_speeds)
            pwm_values = motor.calculate_pwm(wheel_speeds)
            print(f"PWM Values: {pwm_values}")
            Payload.set_motors(shared_data, fl=pwm_values['PWM_FL'], fr=pwm_values['PWM_FR'], rl=pwm_values['PWM_RL'], rr=pwm_values['PWM_RR'])
            gui.start_gui(shared_data, processor, enc_local_map, motor=motor, poll=0.00002)

            # Example: Log encoder and sonar data to their respective logs
            try:
                encoder_data = processor.get_encoder_data()  # Replace with actual method to fetch encoder data
                print(f"Encoder Data: {encoder_data}")  # Log to console instead
            except Exception as exc:
                print(f"Error fetching encoder data: {exc}")

            try:
                sonar_data = shared_data.get("sonar")  # Replace with actual method to fetch sonar data
                print(f"Sonar Data: {sonar_data}")  # Log to console instead
            except Exception as exc:
                print(f"Error fetching sonar data: {exc}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # stop everything
        Payload.set_flags_byte(shared_data, 0b00010000)
        Payload.set_motors(shared_data, 1, 1, 1, 1)  # stop motors
        processor.stop()
        #sonar_proc.stop()
        enc_local_map.stop()
        running_event.clear()
        testingUART.close_serial()


        # Close the figure cleanly to avoid the white/blank window during shutdown
        try:
            if fig  and ani is not None:
                plt.close(fig)
        except Exception:
            pass



if __name__ == "__main__":
    main()