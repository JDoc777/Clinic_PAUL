#pragma once
#include <Arduino.h>
#include "PID.h"

// Last PWM values applied
extern int currentSpeedFL;
extern int currentSpeedFR;
extern int currentSpeedRL;
extern int currentSpeedRR;

// Desired *angular* wheel speeds (rad/s)
extern float desiredOmegaFL;
extern float desiredOmegaFR;
extern float desiredOmegaRL;
extern float desiredOmegaRR;

void initMotorPID(float kp, float ki, float kd);

void stopMotors();

// m values = linear velocity (m/s × 100)
void mecanumDrive(int fl_cmd, int fr_cmd, int rl_cmd, int rr_cmd);

// Must run every 10 ms (dt = 0.01)
void updateMotorPID(float dt);

void applyPWM(int fl_pwm, int fr_pwm, int rl_pwm, int rr_pwm);
