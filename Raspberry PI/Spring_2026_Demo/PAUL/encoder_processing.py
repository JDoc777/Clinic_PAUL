import time
import threading
import math

# Constants
PULSES_PER_REV = 222.5
WHEEL_RADIUS = 0.052

# Encoder state
encoder_state = {
    'FR': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time(), 'history': []},
    'FL': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time(), 'history': []},
    'RR': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time(), 'history': []},
    'RL': {'last_encoder_pos': 0, 'total_ticks': 0, 'last_update_time': time.time(), 'history': []}
}

# Robot position
robot_position = {'x': 0.0, 'y': 0.0, 'angle': 0.0}


def calculate_velocity_components(Vfr, Vfl, Vrr, Vrl):
    R = 0.04
    lx = 0.105
    ly = 0.15

    Vx = (R / 4) * (Vfr + Vfl + Vrr + Vrl)
    Vy = (R / 4) * (Vfr - Vfl + Vrr - Vrl)
    Va = (R / (4 * (lx + ly))) * (Vfr - Vfl - Vrr + Vrl)

    return Vx, Vy, Va


def update_robot_position(Vx, Vy, delta_t, Va):
    global robot_position

    Xold = robot_position['x']
    Yold = robot_position['y']
    theta_old = robot_position['angle']

    # YOUR ORIGINAL SCALING (UNCHANGED)
    Xnew = (Xold + ((Vx * 25) * math.cos(theta_old) - (Vy * 25) * math.sin(theta_old)) * delta_t)
    Ynew = (Yold + ((Vx * 25) * math.sin(theta_old) + (Vy * 25) * math.cos(theta_old)) * delta_t)
    theta_new = theta_old + (Va * 25) * delta_t

    robot_position['x'] = Xnew
    robot_position['y'] = Ynew
    robot_position['angle'] = theta_new

    return robot_position


class EncoderProcessor:
    def __init__(self, shared_data, poll=0.05):
        self.shared = shared_data
        self.poll = poll

        self._running = threading.Event()
        self._running.set()

        self.last_loop_time = time.time()

        self._thread = threading.Thread(target=self._loop, daemon=True)

        self.Vx = 0.0
        self.Vy = 0.0
        self.Va = 0.0

        # ===== LOGGING =====
        self.tick_log = {'FR': [], 'FL': [], 'RR': [], 'RL': []}
        self.time_log = []
        self.start_time = time.time()
        self.log_duration = 4
        self.has_plotted = False

        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            raw = self.shared.get_encoder_raw()
            print(f"Raw encoder data: {raw}", flush=True)

            if raw is not None:
                velocity_fr = self.update_and_calculate_velocity('FR', raw[0])
                velocity_fl = self.update_and_calculate_velocity('FL', raw[1])
                velocity_rr = self.update_and_calculate_velocity('RR', raw[2])
                velocity_rl = self.update_and_calculate_velocity('RL', raw[3])

                Vx, Vy, Va = calculate_velocity_components(
                    velocity_fr, velocity_fl, velocity_rr, velocity_rl
                )

                current_time = time.time()
                delta_t = current_time - self.last_loop_time
                self.last_loop_time = current_time

                update_robot_position(Vx, Vy, delta_t, Va)

                print(f"Encoder Data: FR={raw[0]} FL={raw[1]} RR={raw[2]} RL={raw[3]}", flush=True)
                print(f"Velocities: FR={velocity_fr:.4f} FL={velocity_fl:.4f} RR={velocity_rr:.4f} RL={velocity_rl:.4f}", flush=True)

                # ===== LOG DELTA TICKS =====
                t = current_time - self.start_time
                self.time_log.append(t)


                self.tick_log['FR'].append(encoder_state['FR'].get('last_delta_ticks', 0))
                self.tick_log['FL'].append(encoder_state['FL'].get('last_delta_ticks', 0))
                self.tick_log['RR'].append(encoder_state['RR'].get('last_delta_ticks', 0))
                self.tick_log['RL'].append(encoder_state['RL'].get('last_delta_ticks', 0))
                print(f"Logged ticks at t={t:.2f}s: FR={self.tick_log['FR'][-1]} FL={self.tick_log['FL'][-1]} RR={self.tick_log['RR'][-1]} RL={self.tick_log['RL'][-1]}", flush=True)                

                # ===== AUTO PLOT =====
                if t >= self.log_duration:
                    self.ready_to_plot = True

            time.sleep(self.poll)

    def _plot_ticks(self):
        import matplotlib.pyplot as plt

        wheels = ['FR', 'FL', 'RR', 'RL']

        for w in wheels:
            plt.figure()
            plt.plot(self.time_log, self.tick_log[w])

            plt.xlabel("Time (s)")
            plt.ylabel("Raw Ticks")
            plt.title(f"{w} Encoder Raw Ticks")
            plt.grid()

        plt.show()
    
    def plot_ticks(self):
        self._plot_ticks()

    def get_velocity(self, wheel):
        return encoder_state[wheel].get('last_velocity', 0.0)

    def update_and_calculate_velocity(self, wheel, current_encoder_pos):
        state = encoder_state[wheel]
        current_time = time.time()

        # ===== ADD TO HISTORY FIRST =====
        state['history'].append((current_time, current_encoder_pos))

        WINDOW_SIZE = 5

        if len(state['history']) > WINDOW_SIZE:
            state['history'].pop(0)

        # ===== COMPUTE DELTA TICKS =====
        if len(state['history']) >= WINDOW_SIZE:
            t_old, pos_old = state['history'][0]
            t_new, pos_new = state['history'][-1]

            delta_ticks = pos_new - pos_old
            dt_window = t_new - t_old
        else:
            delta_ticks = 0
            dt_window = 1e-6  # avoid divide by zero

        # ===== STORE FOR PLOTTING =====
        state['last_delta_ticks'] = delta_ticks

        # ===== VELOCITY (CORRECT DT) =====
        prev = state.get('last_velocity', 0.0)

        if len(state['history']) >= WINDOW_SIZE and dt_window > 0:
            velocity = (2 * math.pi * WHEEL_RADIUS * delta_ticks) / (PULSES_PER_REV * dt_window)
        else:
            velocity = state.get('last_velocity', 0.0)

        state['last_velocity'] = velocity
        state['last_encoder_pos'] = current_encoder_pos
        state['last_update_time'] = current_time

        return velocity

    def stop(self, join_timeout=1.0):
        self._running.clear()
        self._thread.join(timeout=join_timeout)


def create_and_run(shared_data, poll=0.05):
    return EncoderProcessor(shared_data, poll=poll)


if __name__ == "__main__":
    print("Run this via run_all.py")