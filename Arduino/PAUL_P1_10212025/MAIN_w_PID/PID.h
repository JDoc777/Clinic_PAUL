#pragma once
#include <Arduino.h>

class PIDController {
public:
    float kp;
    float ki;
    float kd;

    float integral;
    float prevError;

    PIDController();
    PIDController(float kp, float ki, float kd);

    float compute(float setpoint, float measured, float dt);
    void reset();
};
