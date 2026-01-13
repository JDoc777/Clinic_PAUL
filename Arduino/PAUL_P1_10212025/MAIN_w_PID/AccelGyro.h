#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace IMU {
  // Overload 1: keep old usage IMU::begin(Wire)
  bool begin(TwoWire& bus);

  // Overload 2: explicit address
  bool begin(TwoWire& bus, uint8_t addr);

  // Read accelerometer (g) and gyro (deg/s). Returns false on I2C error.
  bool read(float& ax_g, float& az_g, float& ay_g,
            float& gy_dps, float& gz_dps, float& gx_dps);
}

// Kalman-fused tilt (degrees)
struct TiltAngles {
  float roll_deg;   // about X
  float pitch_deg;  // about Y
};

// Start Kalman fusion (performs short stationary gyro calibration).
// Call after IMU::begin(...). Keep device still during this call.
bool IMU_KalmanBegin();

// Update fusion using internal timing; returns true if new angles produced.
bool IMU_KalmanUpdate(TiltAngles& out);

// Optional tuning (defaults: Q_angle=0.001, Q_bias=0.003, R_measure=0.03)
void IMU_KalmanSetTunings(float Q_angle, float Q_bias, float R_measure);
void IMU_KalmanGetTunings(float& Q_angle, float& Q_bias, float& R_measure);
