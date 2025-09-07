PAUL – Arduino Firmware
This directory contains the Arduino Giga R1 firmware for the PAUL (Programmable Autonomous Utility Lift) robot. The Arduino handles all low-level I/O, motor control, and sensor integration, and communicates with the Raspberry Pi over UART.

Responsibilities
-Motor control (PWM → H-bridge drivers for mecanum wheels)
-Servo control for claw arm
-Encoder counting and closed-loop speed/position PID
-Sonar distance measurement (TRIG/ECHO)
-DHT sensor for temperature/humidity
-Status LEDs & user button handling
-UART communication with Raspberry Pi
