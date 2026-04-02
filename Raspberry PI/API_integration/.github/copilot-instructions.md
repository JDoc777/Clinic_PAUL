# AI Coding Agent Instructions for Clinic_PAUL

## Project Overview
This repository contains code for a robotics project focused on obstacle detection, pathfinding, and sensor data processing. The project is organized into Python modules, each handling specific aspects of the system, such as sensor data processing, motor control, and pathfinding.

### Key Components
- **Sensor Data Processing**: Modules like `sonar_processing.py`, `encoder_processing.py`, and `covariance_processing.py` handle data from various sensors.
- **Pathfinding**: The `pathfinding.py` module implements algorithms for navigating through an obstacle grid.
- **Visualization**: The `live_plots.py` and `gyro_plot.py` modules provide real-time data visualization.
- **Simulation**: The `simulate_covariance/` directory contains scripts for simulating and testing covariance calculations.
- **Motor Control**: The `PIDmotor.py` module manages motor control using PID algorithms.

## Developer Workflows

### Running the Project
- Use `run_all.py` to execute the main program that integrates all components.
- Ensure all dependencies are installed before running the script.

### Testing
- Unit tests are located in the `simulate_covariance/test.py` file.
- Run tests using the following command:
  ```bash
  python3 simulate_covariance/test.py
  ```

### Debugging
- Use `testingUART.py` for debugging UART communication.
- Visualization scripts like `live_plots.py` can be used to debug sensor data in real-time.

## Project-Specific Conventions
- **File Naming**: Files are named based on their functionality, e.g., `sonar_processing.py` for sonar data processing.
- **Data Flow**: Sensor data is processed in dedicated modules and passed to higher-level modules like `pathfinding.py`.
- **Simulation**: Use the `simulate_covariance/` directory for testing algorithms in isolation.

## Integration Points
- **External Dependencies**: Ensure Python packages for plotting and numerical computation (e.g., `matplotlib`, `numpy`) are installed.
- **Cross-Component Communication**: Data flows from sensor processing modules to the pathfinding module, which then interacts with motor control.

## Examples
### Adding a New Sensor
1. Create a new module, e.g., `new_sensor_processing.py`.
2. Follow the structure of existing sensor modules like `sonar_processing.py`.
3. Integrate the new module into `run_all.py`.

### Modifying Pathfinding
1. Update the `pathfinding.py` module.
2. Test changes using the obstacle grid processing module (`obstacle_grid_processing.py`).

---

For further questions or clarifications, refer to the module docstrings or contact the repository owner.