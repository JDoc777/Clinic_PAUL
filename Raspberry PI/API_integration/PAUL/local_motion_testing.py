import math
import threading
import time

from networkx import omega
import obstacle_grid_processing
from obstacle_grid_processing import ObstacleGrid
import Payload

# Fake values for testing
fake_goal_x = 1  # Goal position (x)
fake_goal_y = 3.5  # Goal position (y)
fake_fused_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Robot's current position and heading

class local_motion_testing:
    def __init__(self, shared_data, poll=0.05):
        self.shared_data = shared_data
        self.poll = poll
        
        # Register RK in shared_data BEFORE threads start
        setattr(self.shared_data, "local_motion_testing", self)

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

        self.motion_mode = "ACKERMAN"
        self.R_min = 1.0
        self.alpha_max = math.radians(25)
        self.Ky_crab = 1.0


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
                print("No path grid available yet.")
                time.sleep(target_dt)
                Payload.set_motors(self.shared_data, 0, 0, 0, 0)
            else:
                self.update_state()
                self.update_errors()
                self.update_motion_mode()
                Vx, Vy, omega = self.compute_control()
                self.apply_control(Vx, Vy, omega)
                
                """self.FL_PWM = self.convert_linear_to_motor_speed(self.FL)
                self.FR_PWM = self.convert_linear_to_motor_speed(self.FR)
                self.BL_PWM = self.convert_linear_to_motor_speed(self.BL)
                self.BR_PWM = self.convert_linear_to_motor_speed(self.BR)"""

                #Payload.set_motors(self.shared_data, self.FL, self.FR, self.BL, self.BR)


            # Debugging: Print wheel speeds to verify they are updated


            # Sleep to maintain the loop frequency
            time.sleep(max(0, target_dt - (time.time() - start_time)))

    def update_state(self):
    
        self.pose = obstacle_grid_processing.fused_pose

        self.x = -self.pose['x']
        self.y =  self.pose['y']
        self.theta = self.pose['theta']

        self.wp_x = self.obstacle_grid.wp_x
        self.wp_y = self.obstacle_grid.wp_y
        
    def update_errors(self):

        dx = self.wp_x - self.x
        dy = self.wp_y - self.y

        self.x_r =  math.cos(self.theta)*dx + math.sin(self.theta)*dy
        self.y_r = -math.sin(self.theta)*dx + math.cos(self.theta)*dy

        self.heading_error = (math.atan2(dy, dx) - self.theta + math.pi) % (2*math.pi) - math.pi

        self.Ld = math.sqrt(self.x_r*self.x_r + self.y_r*self.y_r)
    
    def update_motion_mode(self):

        if self.Ld < 0.05:
            return

        if abs(self.y_r) < 1e-5:
            R = float('inf')
        else:
            R = (self.Ld*self.Ld) / (2.0*self.y_r)

        if abs(R) < self.R_min:
            self.motion_mode = "TURN"

        elif abs(self.heading_error) < self.alpha_max:
            self.motion_mode = "CRAB"

        else:
            self.motion_mode = "ACKERMAN"

    def compute_control(self):

        if self.motion_mode == "TURN":

            Vx = 0.0
            Vy = 0.0
            omega = self.compute_angular_velocity(self.heading_error)

        elif self.motion_mode == "ACKERMAN":

            v = self.compute_linear_velocity(self.Ld)

            if self.Ld > 1e-5:
                kappa = (2.0*self.y_r)/(self.Ld*self.Ld)
            else:
                kappa = 0.0

            omega = v * kappa

            Vx = v
            Vy = 0.0

        elif self.motion_mode == "CRAB":

            kx = 1.0   # forward gain (tune this)
            Vx = kx * self.x_r

            Vy = self.Ky_crab * self.y_r

            omega = 0.0

        return Vx, Vy, omega

    def calculate_wheel_speeds(self, Vx, Vy, omega):

        L = 0.1

        self.FL = (Vx - Vy - L*omega) * 50
        self.FR = (Vx + Vy + L*omega) * 50
        self.BL = (Vx + Vy - L*omega) * 50
        self.BR = (Vx - Vy + L*omega) * 50

        return self.FL, self.FR, self.BL, self.BR
    
    def apply_control(self, Vx, Vy, omega):

        self.calculate_wheel_speeds(Vx, Vy, omega)
        

        Payload.set_motors(
            self.shared_data,
            self.FL,
            self.FR,
            self.BL,
            self.BR
        )
    
    

    def calculate_velocities(self, distance, delta_angle):
        
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

        return self.v
    
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
    
    """def calculate_wheel_speeds(self, V_linear, V_angular, theta_target):
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
        
        return self.FL, self.FR, self.BL, self.BR, self.V_x, self.V_y"""

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
    return local_motion_testing(shared_data, poll=poll)

def main():
    # Fake shared data (replace with actual shared data if available)
    shared_data = {}

    # Create an instance of ReverseKinematics
    local_motion_testing = create_and_run(shared_data)



    
    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        #print("Stopping ReverseKinematics...")
        local_motion_testing.stop()


def run():
    """Entry point for run_all: runs main() with default settings."""
    main()


if __name__ == "__main__":
    run()





