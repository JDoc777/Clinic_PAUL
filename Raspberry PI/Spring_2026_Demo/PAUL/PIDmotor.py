from encoder_processing import EncoderProcessor
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PIDMotor:
    def __init__(self, encoder_processor):
        self.encoder_processor = encoder_processor
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.integral = {'FR': 0.0, 'FL': 0.0, 'RR': 0.0, 'RL': 0.0}
        self.e_prev = {'FR': 0.0, 'FL': 0.0, 'RR': 0.0, 'RL': 0.0}
        self.pwm_logs = {'FR': [], 'FL': [], 'RR': [], 'RL': []}
        self.velocity_logs = {'FR': [], 'FL': [], 'RR': [], 'RL': []}
        self.time_logs = []


    def velocity_to_pwm(self, velocity, vmax=2.513, pwm_max=255):

        # Clip to avoid exceeding range
        if velocity > vmax:
            velocity = vmax
        elif velocity < -vmax:
            velocity = -vmax
        
        scale = pwm_max / vmax
        pwm = int(round(velocity * scale))
        return pwm



    def display_velocities(self):
        """
        Display the velocity output for all wheels.
        """
        velocities = {
            'FR': self.encoder_processor.get_velocity('FR'),
            'FL': self.encoder_processor.get_velocity('FL'),
            'RR': self.encoder_processor.get_velocity('RR'),
            'RL': self.encoder_processor.get_velocity('RL'),
        }
        for wheel, velocity in velocities.items():
            print(f"Wheel!!!!!! {wheel} Velocity: {velocity:.4f} m/s")

    def set_wheel_speeds(self, speeds):
        """
        Set the speeds for each wheel in meters per second and print them.
        :param speeds: Dictionary with keys 'set_FR', 'set_FL', 'set_RR', 'set_RL' and their respective speeds in m/s.
        """
        set_FR = speeds.get('set_FR', 0.0)
        set_FL = speeds.get('set_FL', 0.0)
        set_RR = speeds.get('set_RR', 0.0)
        set_RL = speeds.get('set_RL', 0.0)

        print(f"Setting FR speed to {set_FR:.4f} m/s")
        print(f"Setting FL speed to {set_FL:.4f} m/s")
        print(f"Setting RR speed to {set_RR:.4f} m/s")
        print(f"Setting RL speed to {set_RL:.4f} m/s")
        # Here you would add logic to send these speeds to the motor controllers if needed.

    def calculate_velocity_error(self, target_speeds):
        """
        Calculate the error between the current velocity and the target velocity for each wheel.
        :param target_speeds: Dictionary with keys 'set_FR', 'set_FL', 'set_RR', 'set_RL' and their respective target speeds in m/s.
        :return: Dictionary with the error for each wheel.
        """
        current_velocities = {
            'FR': self.encoder_processor.get_velocity('FR'),
            'FL': self.encoder_processor.get_velocity('FL'),
            'RR': self.encoder_processor.get_velocity('RR'),
            'RL': self.encoder_processor.get_velocity('RL'),
        }

        errors = {
            'FR': target_speeds.get('set_FR', 0.0) - current_velocities['FR'],
            'FL': target_speeds.get('set_FL', 0.0) - current_velocities['FL'],
            'RR': target_speeds.get('set_RR', 0.0) - current_velocities['RR'],
            'RL': target_speeds.get('set_RL', 0.0) - current_velocities['RL'],
        }

        for wheel, error in errors.items():
            print(f"Error for {wheel}: {error:.4f} m/s")

        return errors

    def set_pid_constants(self, kp, ki, kd):
        """
        Set the PID constants for the motor controller.
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        print(f"PID constants set: kp={kp}, ki={ki}, kd={kd}")


    def calculate_pwm(self, target_speeds):
        """
        Calculate PWM values for each wheel based on the velocity error and PID constants.
        :param target_speeds: Dictionary with keys 'set_FR', 'set_FL', 'set_RR', 'set_RL' and their respective target speeds in m/s.
        :return: Dictionary with PWM values for each wheel.
        """
        errors = self.calculate_velocity_error(target_speeds)

        # Example PID control logic (simplified)
        

        PID_FR = self.PID(self.kp, self.ki, self.kd, target_speeds.get('set_FR', 0.0), self.encoder_processor.get_velocity('FR'), 'FR')
        PID_FL = self.PID(self.kp, self.ki, self.kd, target_speeds.get('set_FL', 0.0), self.encoder_processor.get_velocity('FL'), 'FL')
        PID_RR = self.PID(self.kp, self.ki, self.kd, target_speeds.get('set_RR', 0.0), self.encoder_processor.get_velocity('RR'), 'RR')
        PID_RL = self.PID(self.kp, self.ki, self.kd, target_speeds.get('set_RL', 0.0), self.encoder_processor.get_velocity('RL'), 'RL')
        PWM_FR = self.velocity_to_pwm(PID_FR)
        PWM_FL = self.velocity_to_pwm(PID_FL)
        PWM_RR = self.velocity_to_pwm(PID_RR)
        PWM_RL = self.velocity_to_pwm(PID_RL)

        # Log PWM values and time
        self.pwm_logs['FR'].append(PWM_FR)
        self.pwm_logs['FL'].append(PWM_FL)
        self.pwm_logs['RR'].append(PWM_RR)
        self.pwm_logs['RL'].append(PWM_RL)
        self.time_logs.append(time.time())

        # Log current velocities
        current_velocities = {
            'FR': self.encoder_processor.get_velocity('FR'),
            'FL': self.encoder_processor.get_velocity('FL'),
            'RR': self.encoder_processor.get_velocity('RR'),
            'RL': self.encoder_processor.get_velocity('RL'),
        }
        for wheel in ['FR', 'FL', 'RR', 'RL']:
            self.velocity_logs[wheel].append(current_velocities[wheel])

        print(f"Calculated PID: FR={PID_FR:.2f}, FL={PID_FL:.2f}, RR={PID_RR:.2f}, RL={PID_RL:.2f}")
        print(f"Calculated PWM: FR={PWM_FR:.2f}, FL={PWM_FL:.2f}, RR={PWM_RR:.2f}, RL={PWM_RL:.2f}")

        return {'PWM_FR': PWM_FR, 'PWM_FL': PWM_FL, 'PWM_RR': PWM_RR, 'PWM_RL': PWM_RL}

    def PID(self, Kp, Ki, Kd, setpoint, measurement, wheel):
        """
        Perform PID calculations to compute the manipulated variable (MV).
        :param Kp: Proportional gain.
        :param Ki: Integral gain.
        :param Kd: Derivative gain.
        :param setpoint: Desired target value.
        :param measurement: Current measured value.
        :param wheel: The wheel identifier ('FR', 'FL', 'RR', 'RL').
        :return: Manipulated variable (MV).
        """
        # Value of offset - when the error is equal to zero
        offset = 0

        # Get the last update time for the wheel
        last_update_time = self.encoder_processor.get_last_update_time(wheel)
        current_time = time.time()

        # PID calculations
        e = setpoint - measurement
        P = Kp * e
        self.integral[wheel] += Ki * e * (current_time - last_update_time)
        D = Kd * (e - self.e_prev[wheel]) / (current_time - last_update_time)

        # Calculate manipulated variable - MV
        MV = offset + P + self.integral[wheel] + D

        # Update stored data for the next iteration
        self.e_prev[wheel] = e

        return MV

    def plot_pwm(self):
        """
        Plot the PWM values over time for each wheel.
        """
        if not self.time_logs:
            print("No data to plot.")
            return

        plt.figure(figsize=(10, 8))

        # Plot for each wheel
        for i, wheel in enumerate(['FR', 'FL', 'RR', 'RL'], start=1):
            plt.subplot(2, 2, i)
            plt.plot(self.time_logs, self.pwm_logs[wheel], label=f'{wheel} PWM')
            plt.xlabel('Time (s)')
            plt.ylabel('PWM')
            plt.title(f'{wheel} PWM vs Time')
            plt.legend()
            plt.grid()

        plt.tight_layout()
        plt.show()

    def animate_velocities(self):
        """
        Animate the velocities over time for each wheel.
        """
        if not self.time_logs:
            print("No data to animate.")
            return

        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        wheels = ['FR', 'FL', 'RR', 'RL']
        lines = {}

        # Initialize subplots for each wheel
        for i, wheel in enumerate(wheels):
            ax = axes[i // 2, i % 2]
            ax.set_title(f'{wheel} Velocity vs Time')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Velocity (m/s)')
            ax.grid()
            lines[wheel], = ax.plot([], [], label=f'{wheel} Velocity')
            ax.legend()

        def update(frame):
            # Update the data for each wheel
            for wheel in wheels:
                lines[wheel].set_data(self.time_logs, self.velocity_logs[wheel])
            # Adjust x and y limits dynamically
            for ax, wheel in zip(axes.flatten(), wheels):
                ax.relim()
                ax.autoscale_view()
            return lines.values()

        ani = FuncAnimation(fig, update, interval=100, blit=False)
        plt.tight_layout()
        plt.show()

# Example usage
if __name__ == "__main__":
    # Assuming shared_data is initialized elsewhere
    # ...existing code...
    shared_data = None  # Replace with actual shared_data initialization
    encoder_processor = EncoderProcessor(shared_data)
    motor = PIDMotor(encoder_processor)

    # Display velocities
    motor.display_velocities()

    # Example: Call calculate_pwm in a loop and then plot
    target_speeds = {'set_FR': 1.0, 'set_FL': 1.0, 'set_RR': 1.0, 'set_RL': 1.0}
    for _ in range(10):  # Simulate 10 iterations
        motor.calculate_pwm(target_speeds)
        time.sleep(0.1)  # Simulate a delay

    # Plot the PWM values
    motor.plot_pwm()

    # Animate the velocity values
    motor.animate_velocities()