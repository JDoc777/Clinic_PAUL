import matplotlib.pyplot as plt
import matplotlib.animation as animation
from testingUART import UARTSharedData, UARTReadThread, open_serial, threading

def plot_gyro_accel():
    shared_data = UARTSharedData()
    running_event = threading.Event()
    running_event.set()

    ser = open_serial(timeout=1)
    read_thread = UARTReadThread(ser, shared_data, running_event)
    read_thread.start()

    fig, (ax_gyro, ax_accel) = plt.subplots(2, 1, figsize=(14, 10))  # Increased figure size
    fig.suptitle("Real-Time Gyro and Accel Data")

    # Initialize data buffers
    gyro_data = {"pitch": [], "yaw": [], "roll": []}
    accel_data = {"surge": [], "heave": [], "sway": []}
    time_data = []

    def update_plot(frame):
        data = shared_data.get_received_data()
        if data:
            # Append new data
            time_data.append(len(time_data))
            gyro_data["pitch"].append(data["gyro"]["pitch"])
            gyro_data["yaw"].append(data["gyro"]["yaw"])
            gyro_data["roll"].append(data["gyro"]["roll"])
            accel_data["surge"].append(data["accel"]["surge"])
            accel_data["heave"].append(data["accel"]["heave"])
            accel_data["sway"].append(data["accel"]["sway"])

            # Limit data to the last 1000 points
            if len(time_data) > 1000:
                time_data.pop(0)
                for key in gyro_data:
                    gyro_data[key].pop(0)
                for key in accel_data:
                    accel_data[key].pop(0)

            # Clear and redraw gyro plot
            ax_gyro.clear()
            ax_gyro.set_title("Gyro Data")
            ax_gyro.plot(time_data, gyro_data["pitch"], label="Pitch")
            ax_gyro.plot(time_data, gyro_data["yaw"], label="Yaw")
            ax_gyro.plot(time_data, gyro_data["roll"], label="Roll")
            ax_gyro.legend()
            ax_gyro.set_ylabel("Radians/sec")
            ax_gyro.set_ylim(0, 2 * 3.14159)  # Set y-axis limits for better visibility

            # Clear and redraw accel plot
            ax_accel.clear()
            ax_accel.set_title("Accel Data")
            ax_accel.plot(time_data, accel_data["surge"], label="Surge")
            ax_accel.plot(time_data, accel_data["heave"], label="Heave")
            ax_accel.plot(time_data, accel_data["sway"], label="Sway")
            ax_accel.legend()
            ax_accel.set_ylabel("Acceleration (m/s²)")
            ax_accel.set_xlabel("Time (frames)")
            ax_accel.set_ylim(-1.5, 1.5)  # Set y-axis limits for better visibility

    ani = animation.FuncAnimation(fig, update_plot, interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        running_event.clear()
        read_thread.join()
        ser.close()
        print("Stopped.")

if __name__ == "__main__":
    plot_gyro_accel()
