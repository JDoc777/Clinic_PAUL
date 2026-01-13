""""
F_obstacle_xy = {'x': 0.0, 'y': 0.0}
R_obstacle_xy = {'x': 0.0, 'y': 0.0}
B_obstacle_xy = {'x': 0.0, 'y': 0.0}
L_obstacle_xy = {'x': 0.0, 'y': 0.0}

temp_angle = robot_position['angle']
        R_sonar_angle = temp_angle - math.radians(90)
        L_sonar_angle = temp_angle + math.radians(90)
        B_sonar_angle = temp_angle - math.radians(180)
        F_sonar_x = temp_x + 0.125 * math.cos(temp_angle)
        F_sonar_y = temp_y + 0.125 * math.sin(temp_angle)
        R_sonar_x = temp_x + 0.1 * math.cos(R_sonar_angle)
        R_sonar_y = temp_y + 0.1 * math.sin(R_sonar_angle)
        L_sonar_x = temp_x + 0.1 * math.cos(L_sonar_angle)
        L_sonar_y = temp_y + 0.1 * math.sin(L_sonar_angle)
        B_sonar_x = temp_x + 0.125 * math.cos(B_sonar_angle)
        B_sonar_y = temp_y + 0.125 * math.sin(B_sonar_angle)
        F_sensor_pos = (F_sonar_x, F_sonar_y)
        R_sensor_pos = (R_sonar_x, R_sonar_y)
        L_sensor_pos = (L_sonar_x, L_sonar_y)
        B_sensor_pos = (B_sonar_x, B_sonar_y)
        F_dist = distance_f / 100
        R_dist = distance_r / 100
        L_dist = distance_l / 100
        B_dist = distance_b / 100
        F_obstacle_x, F_obstacle_y = calculate_obstacle_xy(temp_angle, F_dist, F_sensor_pos)
        R_obstacle_x, R_obstacle_y = calculate_obstacle_xy(R_sonar_angle, R_dist, R_sensor_pos)
        L_obstacle_x, L_obstacle_y = calculate_obstacle_xy(L_sonar_angle, L_dist, L_sensor_pos)
        B_obstacle_x, B_obstacle_y = calculate_obstacle_xy(B_sonar_angle, B_dist, B_sensor_pos)
        F_obstacle_xy = {'x': F_obstacle_x, 'y': F_obstacle_y}
        R_obstacle_xy = {'x': R_obstacle_x, 'y': R_obstacle_y}
        L_obstacle_xy = {'x': L_obstacle_x, 'y': L_obstacle_y}
        B_obstacle_xy = {'x': B_obstacle_x, 'y': B_obstacle_y}
        F_position_queue.put(F_obstacle_xy)
        R_position_queue.put(R_obstacle_xy)
        L_position_queue.put(L_obstacle_xy)
        B_position_queue.put(B_obstacle_xy)

def calculate_obstacle_xy(angle, distance, sensor_position):
    """
"""
    Calculate the (x, y) position of an obstacle detected by a sonar sensor.

    Parameters:
        angle (float): The angle (in degrees) at which the obstacle is detected relative to the sensor.
        distance (float): The distance to the obstacle from the sonar sensor.
        sensor_position (tuple): The (x, y) position of the sonar sensor.

    Returns:
        tuple: The (x, y) coordinates of the detected obstacle.
    """

"""print_queue.put(f"debug distance {distance}")
    # Convert the angle to radians
    angle_rad = angle

    # Calculate the relative (x, y) position of the obstacle
    relative_x = distance * math.cos(angle_rad)
    relative_y = distance * math.sin(angle_rad)
    print_queue.put(f"debug relative pos {relative_x}, {relative_y}")

    # Calculate the absolute (x, y) position of the obstacle
    sensor_x, sensor_y = sensor_position
    print_queue.put(f"debug pos {sensor_x}, {sensor_y}")
    obstacle_x = sensor_x + relative_x
    obstacle_y = sensor_y + relative_y

    return obstacle_x, obstacle_y
"""