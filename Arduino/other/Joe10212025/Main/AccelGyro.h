#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "PinDefinitions.h"   // expects MPU_ADDR (e.g., 0x68)

namespace IMU {

/// Call once after Wire.begin() on the bus the MPU is wired to (you use Wire).
bool begin(TwoWire& bus = Wire, uint8_t addr = MPU_ADDR);

/// Read scaled values: accel in g, gyro in deg/s.
/// Returns true on success, false on I2C read failure (then outputs are zeroed).
bool read(float& ax_g, float& ay_g, float& az_g, float& gx_dps, float& gy_dps, float& gz_dps);

} // namespace IMU