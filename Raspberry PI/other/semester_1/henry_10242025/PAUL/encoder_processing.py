import time
import threading
import math

# Constants
PULSES_PER_REV = 111.25  # Ticks per wheel rotation
WHEEL_RADIUS = 0.04  # Radius in meters (40 mm)

# Encoder state setup
encoder_state = {
    'FR': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time()},
    'FL': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time()},
    'RR': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time()},
    'RL': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time()}
}

# Robot position initialization
robot_position = {'x': 0.0, 'y': 0.0, 'angle': 0.0}  # Initial position (x, y) and angle (in radians)

def calculate_velocity_components(Vfr, Vfl, Vrr, Vrl):
    """Compute robot-centric Vx, Vy, and angular velocity Va from wheel velocities."""
    # Wheel radius
    R = 0.04
    
    # Half the distance between the wheels
    lx = 0.12
    ly = 0.0725
    
    # Calculate velocity components
    Vx = (R / 4) * (Vfr + Vfl + Vrr + Vrl)
    Vy = (R / 4) * (Vfr - Vfl + Vrr - Vrl)
    Va = (R / (4 * (lx + ly))) * (Vfr - Vfl - Vrr + Vrl)
    
    return Vx, Vy, Va

def update_robot_position(Vx, Vy, delta_t, Va):
    """Update the robot's position based on velocity components."""
    global robot_position

    # Current position
    Xold = robot_position['x']
    Yold = robot_position['y']
    theta_old = robot_position['angle']

    # Calculate new position
    Xnew = (Xold + ((Vx*25) * math.cos(theta_old) - (Vy*25) * math.sin(theta_old)) * delta_t)
    Ynew = (Yold + ((Vx*25) * math.sin(theta_old) + (Vy*25) * math.cos(theta_old)) * delta_t)
    theta_new = theta_old + (Va * 25) * delta_t

    # Update the global position
    robot_position['x'] = Xnew
    robot_position['y'] = Ynew
    robot_position['angle'] = theta_new

    return robot_position

class EncoderProcessor:
    """Object that reads encoder tuple from a testingUART.UARTSharedData instance."""

    def __init__(self, shared_data, poll=0.05):
        self.shared = shared_data
        self.poll = poll
        self._running = threading.Event()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            raw = self.shared.get_encoder_raw()
            if raw is not None:
                # Update total ticks and calculate velocity for each wheel
                velocity_fr = self.update_and_calculate_velocity('FR', raw[0])
                velocity_fl = self.update_and_calculate_velocity('FL', raw[1])
                velocity_rr = self.update_and_calculate_velocity('RR', raw[2])
                velocity_rl = self.update_and_calculate_velocity('RL', raw[3])

                # Calculate Vx, Vy, and Va
                Vx, Vy, Va = calculate_velocity_components(velocity_fr, velocity_fl, velocity_rr, velocity_rl)

                # Update robot position
                delta_t = self.poll  # Use the polling interval as delta_t
                robot_position_updated = update_robot_position(Vx, Vy, delta_t, Va)

                # Print raw encoder data, total ticks, velocities, velocity components, and updated position
                #print(f"Encoder Data (raw): FR={raw[0]} FL={raw[1]} RR={raw[2]} RL={raw[3]}", flush=True)
                #print(f"Total Ticks: FR={encoder_state['FR']['total_ticks']} FL={encoder_state['FL']['total_ticks']} "
                print(f"Velocities (m/s): FR={velocity_fr:.4f} FL={velocity_fl:.4f} RR={velocity_rr:.4f} RL={velocity_rl:.4f}", flush=True)
                print(f"Robot Velocity Components: Vx={Vx:.4f} m/s Vy={Vy:.4f} m/s Va={Va:.4f} rad/s", flush=True)
                print(f"Updated Robot Position: x={robot_position_updated['x']:.4f}, y={robot_position_updated['y']:.4f}, angle={robot_position_updated['angle']:.4f}", flush=True)
            time.sleep(self.poll)

    def get_velocity(self, wheel):
        """
        Retrieve the last calculated velocity for a given wheel.
        :param wheel: The wheel identifier ('FR', 'FL', 'RR', 'RL').
        :return: The velocity of the specified wheel in m/s.
        """
        global encoder_state
        return encoder_state[wheel].get('last_velocity', 0.0)

    def update_and_calculate_velocity(self, wheel, current_encoder_pos):
        """Update the total tick count and calculate velocity for a given wheel."""
        global encoder_state
        state = encoder_state[wheel]
        last_pos = state['last_encoder_pos']
        last_time = state['last_update_time']

        # Get the current time
        current_time = time.time()

        # Calculate delta_t (time difference)
        delta_t = current_time - last_time
        if delta_t <= 0:
            return 0.0  # Avoid division by zero or invalid delta_t

        # Calculate the tick difference, accounting for wrap-around
        delta_ticks = current_encoder_pos - last_pos
        if delta_ticks > PULSES_PER_REV / 2:
            delta_ticks -= PULSES_PER_REV
        elif delta_ticks < -PULSES_PER_REV / 2:
            delta_ticks += PULSES_PER_REV

        # Update total ticks and last position
        state['total_ticks'] += delta_ticks
        state['last_encoder_pos'] = current_encoder_pos
        state['last_update_time'] = current_time

        # Calculate velocity using the provided formula
        velocity = (2 * math.pi * WHEEL_RADIUS * delta_ticks) / (PULSES_PER_REV * delta_t)
        velocity = velocity  # Apply scaling factor

        # Store the last calculated velocity
        state['last_velocity'] = velocity
        return velocity

    def get_last_update_time(self, wheel):
        """
        Retrieve the last update time for a given wheel.
        :param wheel: The wheel identifier ('FR', 'FL', 'RR', 'RL').
        :return: The last update time (in seconds since epoch).
        """
        global encoder_state
        return encoder_state[wheel].get('last_update_time', 0.0)

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)

# helper for run_both.py to call
def create_and_run(shared_data, poll=0.05):
    return EncoderProcessor(shared_data, poll=poll)

# If you run this file standalone (not recommended when testingUART is running elsewhere),
# you could start a reader here. But prefer run_both.py orchestrator.
if __name__ == "__main__":
    print("Run this via run_both.py so it gets a shared_data instance from testingUART.")
