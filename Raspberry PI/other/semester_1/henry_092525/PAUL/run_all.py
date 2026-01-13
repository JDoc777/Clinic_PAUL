# single-process orchestrator: start uart reader then start encoder processor (no double serial open)
import testingUART
import encoder_processing
import sonar_processing
import Payload
import encoder_local_map
import time
import matplotlib.pyplot as plt  # added to allow plt.pause() and plt.close()
import matplotlib.animation as ani


def main():

    Rst = 0

    # start the UART reader in background; it owns the serial port
    shared_data, running_event, ser = testingUART.start_reader(start_writer=True, do_handshake=True)

    if Rst==0:
        Payload.set_flags_byte(shared_data, 0b00000000)  # example: set flags directly (LED1 + RESET)
        Rst = 1
        
    else:
        pass
    

    processor = encoder_processing.create_and_run(shared_data, poll=0.05)

    sonar_proc = sonar_processing.create_and_run(shared_data, poll=0.05)


    enc_local_map = encoder_local_map.create_and_run(poll=0.05, debug=True)




    # start local map updater that uses the same shared_data



    Payload.set_text(shared_data, "Starting in 5 seconds")  # example: send text to Arduino
    Payload.set_text(shared_data, "ITS GO TIME")  # example: send text to Arduino
    Payload.set_flags_byte(shared_data, 0b00000100)  # example: set flags directly (LED1 + RESET)
    # Flags byte layout (bits MSB->LSB: b7 b6 b5 b4 b3 b2 b1 b0)
    # b7:RESERVED b6:RESERVED b5:LED1 b4:RESET b3:LED0 b2:MOTOR_ENABLE b1:SENSOR_TARE b0:HEARTBEAT

    Payload.set_motors(shared_data, fl=200, fr=200, rl=200, rr=210)  # example: set all motors forward at 100
    #Payload.set_servos(shared_data, 90, 90, 45, 135)  # example: set both servos to neutral (90 degrees)
   
    # ensure fig and ani exist even if the live-plot line is commented out
    fig = None
    ani = None

    # call the live-plot initializer (can be commented out safely now)
    fig, ani = encoder_local_map.start_live_plot(enc_local_map, interval_ms=1, history_len=5000)

    try:
        # Use plt.pause when a figure exists so GUI events are processed.
        # This keeps the animation responsive and prevents the white-box-on-exit symptom.
        while running_event.is_set():
            if fig  and ani is not None:
                # short pause lets the GUI event loop run; adjust interval as needed
                plt.pause(0.1)
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # stop everything
        Payload.set_flags_byte(shared_data, 0b00010000)
        Payload.set_motors(shared_data, 1, 1, 1, 1)  # stop motors
        processor.stop()
        sonar_proc.stop()
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