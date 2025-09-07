# Clinic_PAUL
PAUL is a full-scale robotics project developed for Rowan University’s ECE Clinic. The robot combines mechanical design, embedded systems, and AI-powered autonomy into one cohesive platform.

Core Features
-Mecanum-wheel drive for full omnidirectional movement.
-Arduino Giga R1 for real-time I/O, motor control, and servo actuation.
-Raspberry Pi 5 for high-level computation: SLAM, mapping, path planning, and AI integration.
-Custom electronics: power distribution board, wiring harness, and sensor/motor driver hats.
-Perception stack: sonar array, encoders, IMU, camera, and DHT sensors for environment awareness.
-Claw arm with multiple degrees of freedom, controlled via Arduino and Pi coordination.
-ChatGPT API integration for natural voice commands and obstacle/object recognition.

System Architecture
-Power: 4S LiPo → protection module → buck/boost converters → distribution rails (12 V & 5 V).
-Control Hierarchy:
 -Arduino Giga R1 → Handles sensors, actuators, safety relays.
 -Raspberry Pi 5 → Maps environment, plans routes, interprets commands.
 -UART bridge → Syncs data between Pi and Arduino.
-Software: C++ firmware (Arduino) + Python (Raspberry Pi), structured into modular components for clarity and testing.

Goals
-Develop a modular robotics platform that demonstrates embedded control, perception, and autonomy.
-Enable real-time navigation, manipulation, and human interaction in lab test courses.
-Provide a foundation for future robotics research and student projects at Rowan.
