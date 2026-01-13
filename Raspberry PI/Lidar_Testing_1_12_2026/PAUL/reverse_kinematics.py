import math
import threading
import time
import obstacle_grid_processing
from obstacle_grid_processing import ObstacleGrid
import Payload

# Fake values for testing
fake_goal_x = 1  # Goal position (x)
fake_goal_y = 3.5  # Goal position (y)
fake_fused_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Robot's current position and heading

class ReverseKinematics:
    def __init__(self, shared_data, poll=0.05):
        self.shared_data = shared_data
        self.poll = poll
        
        # Register RK in shared_data BEFORE threads start
        setattr(self.shared_data, "reverse_kinematics", self)

        self._running = threading.Event()
        self._running.set()

        self.distance = 0.0
        self.delta_angle = 0.0
        self.theta_target = 0.0
        self.V_linear = 0.0
        self.V_angular = 0.0
        self.v = 0.0
        self.omega = 0.0

        self.FL = 0.0
        self.FR = 0.0
        self.BL = 0.0
        self.BR = 0.0
        self.V_x = 0.0
        self.V_y = 0.0

        self.max_speed = 0
        self.dead_zone_limit = 0
        self.motor_speed = 0
        self.adjusted_speed = 0
        self.radius = 0.0
        self.wheel_base = 0.0

        self.FL_PWM = 0
        self.FR_PWM = 0
        self.BL_PWM = 0
        self.BR_PWM = 0 

        self.FL = 0.0
        self.FR = 0.0
        self.BL = 0.0
        self.BR = 0.0

        self.path_grid = None

        self.V_MAX = 0       
        self.V_MIN = 0
        self.SLOW_RADIUS = 0

        self.prev_error_angle = 0.0
        self.dt = 0.0


        self._last_time = time.time()

        # Create obstacle grid
        self.obstacle_grid = ObstacleGrid(shared_data, poll)

        # START THREAD LAST
        self._thread = threading.Thread(target=self.loop, daemon=True)
        self._thread.start()

    def loop(self):
        target_dt = 0.5  # 20 Hz loop frequency
        self._last_time = time.time()

        while self._running.is_set():
            start_time = time.time()
            now = time.time()
            self.dt = now - self._last_time
            self._last_time = now

            if self.dt <= 0 or self.dt > 0.1:
                continue  # Skip this iteration if dt is invalid

            #print("HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIi")

            # Calculate distance, heading error, and velocities

            if self.obstacle_grid.navigation_done:
                print("Navigation is done. Stopping motors.")
                Payload.set_motors(self.shared_data, 0, 0, 0, 0)
                time.sleep(target_dt)
                continue

            self.path_grid = self.obstacle_grid.current_path_grid

            if self.path_grid is None:
                #print("No path grid available yet.")
                time.sleep(target_dt)
                Payload.set_motors(self.shared_data, 0, 0, 0, 0)
            else:
                self.calculate_distance_and_heading_error(self.obstacle_grid.wp_x, self.obstacle_grid.wp_y, obstacle_grid_processing.fused_pose)
                #self.calculate_velocities(self.distance, self.delta_angle)
                self.compute_linear_velocity(self.distance)
                self.compute_angular_velocity(self.delta_angle)
                self.calculate_wheel_speeds(self.v, self.omega, self.theta_target)
                """self.FL_PWM = self.convert_linear_to_motor_speed(self.FL)
                self.FR_PWM = self.convert_linear_to_motor_speed(self.FR)
                self.BL_PWM = self.convert_linear_to_motor_speed(self.BL)
                self.BR_PWM = self.convert_linear_to_motor_speed(self.BR)"""

                #Payload.set_motors(self.shared_data, self.FL, self.FR, self.BL, self.BR)


            # Debugging: Print wheel speeds to verify they are updated


            # Sleep to maintain the loop frequency
            time.sleep(max(0, target_dt - (time.time() - start_time)))

    def calculate_distance_and_heading_error(self, goal_x, goal_y, temp1_position):
        x = -temp1_position['x']
        y = temp1_position['y']
        theta_current = temp1_position['theta']
        
        # Calculate distance
        self.distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        
        # Calculate target angle
        self.theta_target = math.atan2(goal_y - y, goal_x - x)
        
        # Calculate heading error
        self.delta_angle = (self.theta_target - theta_current + math.pi) % (2 * math.pi) - math.pi

        #print ("Distance to goal:", self.distance, "Heading error (radians):", self.delta_angle)
        
        return self.distance, self.delta_angle, self.theta_target
    
    def compute_linear_velocity(self, distance):
        self.V_MAX = 0.25      # max translational velocity (m/s)
        self.V_MIN = 0.03      # minimum useful velocity (m/s)
        self.SLOW_RADIUS = 0.1 # start slowing down when within 1 meter
        #print("Distance !!! to goal inside compute_linear_velocity:", distance)

        if distance < self.SLOW_RADIUS:
            # far away → go at max spee
            self.v = (distance / self.SLOW_RADIUS) * self.V_MAX
        else: 
            self.v = self.V_MAX



        self.v = max(self.V_MIN, self.v)

        return self.v
    
    def compute_angular_velocity(self, delta_angle):
        W_MAX = 1.2
        k_angle = 1.2  # less aggressive

        # Soft turn braking
        if abs(delta_angle) < math.radians(10):
            self.omega = k_angle * delta_angle * 0.3
        else:
            self.omega = k_angle * delta_angle

        # Bound
        self.omega = max(-W_MAX, min(W_MAX, self.omega))

        # Kill tiny drift
        if abs(self.omega) < 0.05:
            self.omega = 0

        return self.omega

    """def calculate_velocities(self, distance, delta_angle):
        
        # add logic to create kd and kangle
        self.k_d = 1
        self.k_angle = 1
        
        self.V_linear = self.k_d * distance
        self.V_angular = self.k_angle * delta_angle
        
        print("Calculated Velocities - Linear:", self.V_linear, "Angular:", self.V_angular)
        return self.V_linear, self.V_angular
    
    def compute_linear_velocity(self, distance):
        # SIMPLE and SAFE controller
        V_MAX = 0.25
        STOP_RADIUS = 0.10      # stop if within 10 cm
        SLOW_RADIUS = 0.30      # start slowing within 30 cm

        # stop if very close
        if distance < STOP_RADIUS:
            self.v = 0.0
            return self.v

        # smooth slowdown (linear)
        if distance < SLOW_RADIUS:
            scale = distance / SLOW_RADIUS
            self.v = V_MAX * scale
        else:
            self.v = V_MAX

        return self.v"""
    
    def compute_angular_velocity(self, delta_angle):
        # ---- clean P-only heading controller ----
        Kp = 0.8            # start here; you can tune between 0.5–1.0
        W_MAX = 0.8         # max turn speed (rad/s)
        DEAD_BAND = math.radians(3)  # ignore tiny angle errors

        # small errors → don't bother turning (reduces jitter)
        if abs(delta_angle) < DEAD_BAND:
            self.omega = 0.0
            return self.omega

        # proportional turn rate
        omega = Kp * delta_angle

        # clamp to ±W_MAX
        omega = max(-W_MAX, min(W_MAX, omega))

        self.omega = omega
        #print("Computed Angular Velocity:", self.omega)
        return self.omega
    
    def calculate_wheel_speeds(self, V_linear, V_angular, theta_target):
        L = 0.1
        
        # Convert linear velocity into x and y components
        self.V_x = V_linear * math.cos(theta_target)
        self.V_y = V_linear * math.sin(theta_target)
        
        # Mecanum wheel equations
        self.FL = (self.V_x - self.V_y - L * V_angular) * 100
        self.FR = (self.V_x + self.V_y + L * V_angular) * 100
        self.BL = (self.V_x + self.V_y - L * V_angular) * 100
        self.BR = (self.V_x - self.V_y + L * V_angular) * 100

        #print(f"Wheel Speeds!!!! - FL: {self.FL:.2f}, FR: {self.FR:.2f}, BL: {self.BL:.2f}, BR: {self.BR:.2f}, Vx: {self.V_x:.2f}, Vy: {self.V_y:.2f}")
        
        return self.FL, self.FR, self.BL, self.BR, self.V_x, self.V_y

    def convert_linear_to_motor_speed(self, v_linear):
        self.radius = 0.04
        self.dead_zone_limit = 100  # Define the dead zone range (-175 to 175)
        self.max_speed = 255  # Max motor speed

        self.omega = v_linear / self.radius
        self.omega_max = self.V_MAX / self.radius
        self.motor_speed = int((self.omega / self.omega_max) * self.max_speed)
        self.motor_speed = max(-self.max_speed, min(self.max_speed, self.motor_speed))

        if self.motor_speed > 0:
            self.adjusted_speed = self.motor_speed * ((self.max_speed - self.dead_zone_limit) / self.max_speed) + self.dead_zone_limit
        elif self.motor_speed < 0:
            self.adjusted_speed = self.motor_speed * ((-self.max_speed + self.dead_zone_limit) / -self.max_speed) - self.dead_zone_limit
        else:
            self.adjusted_speed = 0
        self.adjusted_speed = max(-self.max_speed, min(self.adjusted_speed, self.max_speed))
        return int(self.adjusted_speed)

def create_and_run(shared_data, poll=0.005):
    return ReverseKinematics(shared_data, poll=poll)

def main():
    # Fake shared data (replace with actual shared data if available)
    shared_data = {}

    # Create an instance of ReverseKinematics
    reverse_kinematics = create_and_run(shared_data)



    
    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        #print("Stopping ReverseKinematics...")
        reverse_kinematics.stop()


def run():
    """Entry point for run_all: runs main() with default settings."""
    main()


if __name__ == "__main__":
    run()





