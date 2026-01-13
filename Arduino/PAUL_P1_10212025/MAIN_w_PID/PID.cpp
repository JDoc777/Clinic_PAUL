#include "PID.h"

PIDController::PIDController()
    : kp(0), ki(0), kd(0), integral(0), prevError(0) {}

PIDController::PIDController(float kp_, float ki_, float kd_)
    : kp(kp_), ki(ki_), kd(kd_), integral(0), prevError(0) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    integral += error * dt;
    float derivative = (dt > 0) ? ((error - prevError) / dt) : 0;

    prevError = error;

    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
    integral = 0;
    prevError = 0;
}
