#include <Arduino.h>
#include "PinDefinitions.h"
#include "AccelGyro.h"

//standard interval = 
/*
   I2C MPU-6050 Declaration
inline constexpr uint8_t MPU_ADDR = 0x68;
inline constexpr uint8_t MPU6050_PWR0 = 0x6B;
*/

// --- Registers ---
static constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
static constexpr uint8_t REG_ACCEL_XOUT  = 0x3B; // 14-byte burst: AX..AZ, TEMP, GX..GZ

// --- Scale factors for default ranges: ±2 g and ±250 dps ---
static constexpr float ACCEL_LSB_PER_G   = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS  = 131.0f;

static TwoWire* s_bus  = &Wire;
#ifndef MPU_ADDR
#define MPU_ADDR 0x68
#endif
static uint8_t  s_addr = MPU_ADDR;

namespace IMU {

static inline int16_t be16(const uint8_t* b) { return (int16_t)((b[0] << 8) | b[1]); }

bool begin(TwoWire& bus, uint8_t addr) {
  s_bus  = &bus;
  s_addr = addr;

  // Wake device (clear SLEEP bit)
  s_bus->beginTransmission(s_addr);
  s_bus->write(REG_PWR_MGMT_1);
  s_bus->write(0x00);
  if (s_bus->endTransmission() != 0) return false;
  delay(50);
  return true;
}

bool read(float& ax_g, float& ay_g, float& az_g,
          float& gx_dps, float& gy_dps, float& gz_dps)
{
  uint8_t raw[14];

  // Set register pointer, repeated start, read 14 bytes
  s_bus->beginTransmission(s_addr);
  s_bus->write(REG_ACCEL_XOUT);
  if (s_bus->endTransmission(false) != 0) {
    ax_g = ay_g = az_g = gx_dps = gy_dps = gz_dps = 0.0f;
    return false;
  }

  const uint8_t need = 14;
  uint8_t got = s_bus->requestFrom((int)s_addr, (int)need);
  if (got != need) {
    ax_g = ay_g = az_g = gx_dps = gy_dps = gz_dps = 0.0f;
    return false;
  }
  for (uint8_t i = 0; i < need; ++i) raw[i] = (uint8_t)s_bus->read();

  const int16_t ax = be16(&raw[0]);
  const int16_t ay = be16(&raw[2]);
  const int16_t az = be16(&raw[4]);
  // int16_t temp = be16(&raw[6]); // available if needed later
  const int16_t gx = be16(&raw[8]);
  const int16_t gy = be16(&raw[10]);
  const int16_t gz = be16(&raw[12]);

  ax_g   = (float)ax / ACCEL_LSB_PER_G;
  ay_g   = (float)ay / ACCEL_LSB_PER_G;
  az_g   = (float)az / ACCEL_LSB_PER_G;

  gx_dps = (float)gx / GYRO_LSB_PER_DPS;   // roll rate (about X)
  gy_dps = (float)gy / GYRO_LSB_PER_DPS;   // pitch rate (about Y)
  gz_dps = (float)gz / GYRO_LSB_PER_DPS;   // yaw rate (about Z)
  return true;
}

} // namespace IMU
