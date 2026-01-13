//CHAT CODE, just something to work off of

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace PCAServo {

// Global config for all servos on this PCA
struct Config {
  float    freq_hz   = 50.0f;  // 50–60 Hz typical
  uint16_t min_us    = 650;    // tune to your servos
  uint16_t max_us    = 2350;   // tune to your servos
};

// Call once at boot. You can pass Wire1/Wire2 if the PCA is on another bus.
bool begin(TwoWire& bus = Wire, uint8_t addr = 0x40, const Config& cfg = {});

// Provide the mapping from logical servo index -> PCA channel number.
// Example: channels = {15,14,13,12,11}, count = 5.
void attachMap(const uint8_t* channels, uint8_t count);

// Angle setters (degrees 0–180, clamped).
void setAngle(uint8_t logical_index, float deg);
void setAngles(const float* degs, uint8_t count);

// Convenience for 8-bit packed degrees (0–180 expected).
void setPackedDegrees(const uint8_t* packed_deg, uint8_t count);

// Optional: directly set a pulse in microseconds for a logical servo.
void writeMicros(uint8_t logical_index, uint16_t us);

// Kill pulses on all mapped channels (not strictly needed for hobby servos).
void off();

// Utilities
uint16_t angleToMicros(float deg);
uint16_t microsToTicks(uint16_t us);

} // namespace PCAServo
