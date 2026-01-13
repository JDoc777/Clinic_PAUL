from encoder_processing import robot_position
from sonar_processing import SonarProcessor
import math
import threading

class SonarObstacleLocalMap:
    def __init__(self, sonar_processor, shared_data=None, poll=0.05):
        self.shared = shared_data if shared_data is not None else sonar_processor.shared
        self.poll = poll
        self._running = threading.Event()
        self.F_raw = 0
        self.B_raw = 0
        self.L_raw = 0
        self.R_raw = 0
        self._running.set()
        self.sonar_processor = sonar_processor
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running.is_set():
            self.print_combined_data()
            self.sonar_angle_processing()
            self.sonar_axis_processing()
            self.sonar_pos_processing()
            self.sonar_obstacle_axis_processing()
            threading.Event().wait(self.poll)

    def print_combined_data(self):
        sonar_data = self.sonar_processor.shared.get_received_data()
        if sonar_data and "sonar" in sonar_data:
            sonar = sonar_data.get("sonar", {})
            print(f"Robot Position for local map: x={robot_position['x']:.4f}, y={robot_position['y']:.4f}, angle={robot_position['angle']:.4f}")
            print(f"Sonar Data local map: F={sonar.get('F')} B={sonar.get('B')} L={sonar.get('L')} R={sonar.get('R')}")
            self.F_raw = sonar.get('F')
            self.B_raw = sonar.get('B')
            self.L_raw = sonar.get('L')
            self.R_raw = sonar.get('R')

    def sonar_angle_processing(self):
        self.R_sonar_angle = robot_position['angle'] - math.radians(90)
        self.L_sonar_angle = robot_position['angle'] + math.radians(90)
        self.B_sonar_angle = robot_position['angle'] - math.radians(180)
        print(f"R_sonar_angle: {self.R_sonar_angle}")
        print(f"L_sonar_angle: {self.L_sonar_angle}")
        print(f"B_sonar_angle: {self.B_sonar_angle}")

    def sonar_axis_processing(self):
        self.F_sonar_x = robot_position['x'] + 0.125 * math.cos(robot_position['angle'])
        self.F_sonar_y = robot_position['y'] + 0.125 * math.sin(robot_position['angle'])
        self.R_sonar_x = robot_position['x'] + 0.1 * math.cos(self.R_sonar_angle)
        self.R_sonar_y = robot_position['y'] + 0.1 * math.sin(self.R_sonar_angle)
        self.L_sonar_x = robot_position['x'] + 0.1 * math.cos(self.L_sonar_angle)
        self.L_sonar_y = robot_position['y'] + 0.1 * math.sin(self.L_sonar_angle)
        self.B_sonar_x = robot_position['x'] + 0.125 * math.cos(self.B_sonar_angle)
        self.B_sonar_y = robot_position['y'] + 0.125 * math.sin(self.B_sonar_angle)
        print(f"F_sonar: x={self.F_sonar_x:.4f} m, y={self.F_sonar_y:.4f} m")
        print(f"R_sonar: x={self.R_sonar_x:.4f} m, y={self.R_sonar_y:.4f} m")
        print(f"L_sonar: x={self.L_sonar_x:.4f} m, y={self.L_sonar_y:.4f} m")
        print(f"B_sonar: x={self.B_sonar_x:.4f} m, y={self.B_sonar_y:.4f} m")

    def sonar_pos_processing(self):
        self.F_sensor_pos = (self.F_sonar_x, self.F_sonar_y)
        self.R_sensor_pos = (self.R_sonar_x, self.R_sonar_y)
        self.L_sensor_pos = (self.L_sonar_x, self.L_sonar_y)
        self.B_sensor_pos = (self.B_sonar_x, self.B_sonar_y)
      



    def calculate_obstacle_positions(self,angle, distance, sensor_pos):
        angle_rad = angle
        # Calculate the relative (x, y) position of the obstacle
        relative_x = distance * math.cos(angle_rad)
        relative_y = distance * math.sin(angle_rad)

        # Calculate the absolute (x, y) position of the obstacle
        sensor_x, sensor_y = sensor_pos
        obstacle_x = sensor_x + relative_x
        obstacle_y = sensor_y + relative_y

        return obstacle_x, obstacle_y
    
    def sonar_obstacle_axis_processing(self):
        F_obstacle_x, F_obstacle_y = self.calculate_obstacle_positions(robot_position['angle'], (self.F_raw) / 100, self.F_sensor_pos)
        R_obstacle_x, R_obstacle_y = self.calculate_obstacle_positions(robot_position['angle'], (self.R_raw) / 100, self.R_sensor_pos)
        L_obstacle_x, L_obstacle_y = self.calculate_obstacle_positions(robot_position['angle'], (self.L_raw) / 100, self.L_sensor_pos)
        B_obstacle_x, B_obstacle_y = self.calculate_obstacle_positions(robot_position['angle'], (self.B_raw) / 100, self.B_sensor_pos)
        print(f"F_obstacle!!!!!: x={F_obstacle_x:.4f} m, y={F_obstacle_y:.4f} m")
        print(f"R_obstacle!!!!!: x={R_obstacle_x:.4f} m, y={R_obstacle_y:.4f} m")
        print(f"L_obstacle!!!!!: x={L_obstacle_x:.4f} m, y={L_obstacle_y:.4f} m")
        print(f"B_obstacle!!!!!: x={B_obstacle_x:.4f} m, y={B_obstacle_y:.4f} m")

    @staticmethod
    def create_and_run(shared_data=None, sonar_processor=None, poll=0.05, debug=False):
        if sonar_processor is None:
            sonar_processor = SonarProcessor(shared_data, poll=poll)
        return SonarObstacleLocalMap(sonar_processor, shared_data=shared_data, poll=poll)

if __name__ == "__main__":
    print("Run this via a main orchestrator script to provide shared_data instances.")
