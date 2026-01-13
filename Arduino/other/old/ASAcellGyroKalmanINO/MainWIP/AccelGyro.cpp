#include <Arduino.h>
#include <Wire.h>
#include "AccelGyro.h"

// ===================== PRESERVED MPU6050 DECLARATIONS =====================
//standard interval = 
/*
   I2C MPU-6050 Declaration
inline constexpr uint8_t MPU_ADDR = 0x68;
inline constexpr uint8_t MPU6050_PWR0 = 0x6B;
*/

// --- Registers ---
static const uint8_t REG_PWR_MGMT_1  = 0x6B;
static const uint8_t REG_ACCEL_XOUT  = 0x3B; // 14-byte burst: AX..AZ, TEMP, GX..GZ

// --- Scale factors for default ranges: ±2 g and ±250 dps ---
static const float ACCEL_LSB_PER_G   = 16384.0f;
static const float GYRO_LSB_PER_DPS  = 131.0f;

static TwoWire* s_bus  = &Wire;
#ifndef MPU_ADDR
#define MPU_ADDR 0x68
#endif
static uint8_t  s_addr = MPU_ADDR;

// ============================== IMU NAMESPACE =============================
namespace IMU {

static inline int16_t be16(const uint8_t* b) {
  return (int16_t)((uint16_t)b[0] << 8 | (uint16_t)b[1]);
}

// Overload 2: explicit address (primary implementation)
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

// Overload 1: default address (keeps existing code working)
bool begin(TwoWire& bus) {
  return begin(bus, MPU_ADDR);
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

  for (uint8_t i = 0; i < need; ++i) {
    raw[i] = (uint8_t)s_bus->read();
  }

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

// ============================== KALMAN FILTER =============================
// (Internal-only; exports are IMU_Kalman* API — NO setup()/loop() here.)
class _Kalman {
public:
  float Q_angle   = 0.001f; // process noise (angle)
  float Q_bias    = 0.003f; // process noise (gyro bias)
  float R_measure = 0.03f;  // measurement noise (accelerometer)

  float angle = 0.0f; // filtered angle (deg)
  float bias  = 0.0f; // estimated gyro bias (deg/s)
  float rate  = 0.0f; // unbiased rate (deg/s)
  float P[2][2] = {{0,0},{0,0}};

  void setAngle(float a) { angle = a; }

  float update(float measAngleDeg, float gyroRateDps, float dt) {
    // Predict
    rate  = gyroRateDps - bias;
    angle += dt * rate;

    // Covariance prediction (A = [[1,-dt],[0,1]])
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= P[1][1] * dt;
    P[1][1] += Q_bias;

    // Update (H = [1 0])
    const float S  = P[0][0] + R_measure;
    const float K0 = P[0][0] / S;
    const float K1 = P[1][0] / S;
    const float y  = measAngleDeg - angle;

    angle += K0 * y;
    bias  += K1 * y;

    const float P00 = P[0][0], P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;

    return angle;
  }
};

// --------------------------- Local (file-scope) state ----------------------
static _Kalman s_kfRoll, s_kfPitch;
static float   s_gx_bias = 0.0f, s_gy_bias = 0.0f;
static bool    s_kfInited = false;
static unsigned long s_lastMicros = 0;

static inline float _rad2deg(float r){ return r * (180.0f/PI); }

// Accel-only roll/pitch (deg) from g's
// roll  about X: atan2(ay, az)
// pitch about Y: atan2(-ax, sqrt(ay^2 + az^2))
static void _accelAnglesDeg(float ax_g, float ay_g, float az_g,
                            float& rollDeg, float& pitchDeg) {
  rollDeg  = _rad2deg(atan2f(ay_g, az_g));
  pitchDeg = _rad2deg(atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)));
}

// ================================ Public API ===============================
bool IMU_KalmanBegin() {
  // Quick stationary gyro calibration (keep the device still)
  const uint16_t N = 500;
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  float sumGX = 0.0f, sumGY = 0.0f;
  uint16_t got = 0;

  for (uint16_t i = 0; i < N; ++i) {
    if (IMU::read(ax, ay, az, gx, gy, gz)) {
      sumGX += gx;
      sumGY += gy;
      ++got;
    }
    delay(2);
  }
  if (got == 0) return false;

  s_gx_bias = sumGX / got;
  s_gy_bias = sumGY / got;

  // Seed Kalman angles from accel to avoid startup transients
  float rDeg, pDeg;
  _accelAnglesDeg(ax, ay, az, rDeg, pDeg);
  s_kfRoll.setAngle(rDeg);
  s_kfPitch.setAngle(pDeg);

  s_lastMicros = micros();
  s_kfInited = true;
  return true;
}

bool IMU_KalmanUpdate(TiltAngles& out) {
  if (!s_kfInited) return false;

  float ax, ay, az, gx, gy, gz;
  if (!IMU::read(ax, ay, az, gx, gy, gz)) return false;

  // dt in seconds
  const unsigned long now = micros();
  float dt = (now - s_lastMicros) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // clamp to sane range
  s_lastMicros = now;

  // Accel-only angles
  float rollAccDeg, pitchAccDeg;
  _accelAnglesDeg(ax, ay, az, rollAccDeg, pitchAccDeg);

  // Unbiased gyro rates (dps)
  const float gx_dps = gx - s_gx_bias;
  const float gy_dps = gy - s_gy_bias;

  // Kalman fusion
  out.roll_deg  = s_kfRoll.update(rollAccDeg,  gx_dps, dt);
  out.pitch_deg = s_kfPitch.update(pitchAccDeg, gy_dps, dt);
  return true;
}

void IMU_KalmanSetTunings(float Q_angle, float Q_bias, float R_measure) {
  s_kfRoll.Q_angle = s_kfPitch.Q_angle = Q_angle;
  s_kfRoll.Q_bias  = s_kfPitch.Q_bias  = Q_bias;
  s_kfRoll.R_measure = s_kfPitch.R_measure = R_measure;
}

void IMU_KalmanGetTunings(float& Q_angle, float& Q_bias, float& R_measure) {
  Q_angle   = s_kfRoll.Q_angle;
  Q_bias    = s_kfRoll.Q_bias;
  R_measure = s_kfRoll.R_measure;
}
