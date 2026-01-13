//CHAT CODE, just something to work off of

#include "PCAServo.h"

namespace PCAServo {

static Adafruit_PWMServoDriver* s_pca = nullptr;
static TwoWire*    s_bus   = &Wire;
static uint8_t     s_addr  = 0x40;
static Config      s_cfg   {};
static uint8_t     s_map[16];
static uint8_t     s_count = 0;   // how many logical servos we manage (<=16)

// --- internal helpers ---
static inline uint16_t clamp_u16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

uint16_t angleToMicros(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  const float span = (float)(s_cfg.max_us - s_cfg.min_us);
  return (uint16_t)(s_cfg.min_us + (deg / 180.0f) * span);
}

uint16_t microsToTicks(uint16_t us) {
  // PCA9685: 12-bit (0..4095) over one period.
  const float period_us = 1000000.0f / s_cfg.freq_hz;
  const float ticks_f   = ( (float)us / period_us ) * 4096.0f;
  int ticks = (int)(ticks_f + 0.5f);
  return clamp_u16(ticks, 0, 4095);
}

bool begin(TwoWire& bus, uint8_t addr, const Config& cfg) {
  s_bus  = &bus;
  s_addr = addr;
  s_cfg  = cfg;

  // Adafruit lib lets us pass a TwoWire*
  static Adafruit_PWMServoDriver pca_local(s_addr, *s_bus);
  s_pca = &pca_local;

  s_bus->begin();                   // ensure that bus is up
  s_pca->begin();
  s_pca->setPWMFreq(s_cfg.freq_hz); // 50–60 Hz

  // Small settle
  delay(10);
  return true;
}

void attachMap(const uint8_t* channels, uint8_t count) {
  s_count = (count > 16) ? 16 : count;
  for (uint8_t i = 0; i < s_count; ++i) s_map[i] = channels[i];
}

void writeMicros(uint8_t logical_index, uint16_t us) {
  if (!s_pca || logical_index >= s_count) return;
  const uint8_t ch = s_map[logical_index];
  const uint16_t offTicks = microsToTicks(us);
  // Use on=0, off=offTicks
  s_pca->setPWM(ch, 0, offTicks);
}

void setAngle(uint8_t logical_index, float deg) {
  writeMicros(logical_index, angleToMicros(deg));
}

void setAngles(const float* degs, uint8_t count) {
  if (!degs) return;
  const uint8_t n = (count < s_count) ? count : s_count;
  for (uint8_t i = 0; i < n; ++i) setAngle(i, degs[i]);
}

void setPackedDegrees(const uint8_t* packed_deg, uint8_t count) {
  if (!packed_deg) return;
  const uint8_t n = (count < s_count) ? count : s_count;
  for (uint8_t i = 0; i < n; ++i) {
    uint8_t d = packed_deg[i];
    if (d > 180) d = 180;
    setAngle(i, (float)d);
  }
}

void off() {
  if (!s_pca) return;
  for (uint8_t i = 0; i < s_count; ++i) {
    s_pca->setPWM(s_map[i], 0, 0); // 0 width -> off
  }
}

} // namespace PCAServo


