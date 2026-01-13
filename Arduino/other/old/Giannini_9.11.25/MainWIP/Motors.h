#pragma once
#include <Arduino.h>

// --- Current wheel speeds (last commanded) ---
extern int currentSpeedFL;  // Front Left
extern int currentSpeedFR;  // Front Right
extern int currentSpeedRL;  // Rear Left
extern int currentSpeedRR;  // Rear Right

// --- API ---
// Stop all motors (sets speed=0, disables pins)
void stopMotors();

// Drive mecanum wheels with individual wheel speeds (-255..255)
void mecanumDrive(int fl, int fr, int rl, int rr);

// Drive an individual motor
// enablePin: PWM pin
// inPin1/inPin2: direction pins
// speed: -255..255
void driveMotor(int enablePin, int inPin1, int inPin2, int speed);