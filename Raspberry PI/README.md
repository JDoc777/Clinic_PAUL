PAUL – Raspberry Pi Software
This directory contains the Raspberry Pi 5 software stack for the PAUL (Programmable Autonomous Utility Lift) robot. The Pi is responsible for high-level computation, mapping, AI integration, and coordination with the Arduino Giga R1 over UART.

Responsibilities
-Parse and process sensor data from Arduino (sonar, encoders, DHT, status flags)
-Maintain occupancy grid mapping (SLAM)
-Implement pathfinding algorithms (A*, Dijkstra, PID to target)
-Handle motor speed PID and trajectory planning
-Integrate camera input for object detection
-Provide speech recognition and voice command → action translation
-Control claw arm at a high level (open/close, grab/place)

Communication
-Protocol: UART (custom frame format shared with Arduino)
-Direction:
 -Raspberry Pi → Sends motor/claw instructions, high-level commands
 -Arduino → Sends sensor data, actuator feedback

Getting Started
-Clone repo & open /pi/src/main.py.
-Ensure Python 3.11+ is installed.
-Connect Pi UART pins to Arduino Giga R1 (TX↔RX, GND shared).
-Run the software: python3 main.py

Dependencies
-pyserial – UART communication
-numpy, scipy – math & control
-opencv-python – camera & object recognition
-speechrecognition + pyaudio – voice commands
-networkx or python-pathfinding – path planning
