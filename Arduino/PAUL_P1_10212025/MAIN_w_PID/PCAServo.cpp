#include "PCAServo.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire1);

constexpr uint32_t PCA_OSC_HZ = 27000000;
constexpr uint16_t SERVO_MIN_US = 650;   // fully left
constexpr uint16_t SERVO_MAX_US = 2350;  // fully right
constexpr float SERVO_FREQ_HZ   = 50.0f; // standard servo frequency

static constexpr uint8_t CH_BASE     = 15;
static constexpr uint8_t CH_SHOULDER = 14;
static constexpr uint8_t CH_ELBOW    = 13;
static constexpr uint8_t CH_WRIST_P  = 9;
static constexpr uint8_t CH_CLAW     = 11;

static inline uint16_t angleToMicros(uint8_t deg){
  const uint32_t span = (uint32_t)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint16_t)(SERVO_MIN_US + (uint32_t)deg * span / 180UL);
}

static inline uint16_t usToTicks(uint16_t us){
  // ticks = us * freq * 4096 / 1e6
  float t = (float)us * SERVO_FREQ_HZ * 4096.0f / 1000000.0f;
  return (uint16_t)(t + 0.5f);
}

static inline uint8_t clamp180(uint8_t deg) {
  if (deg > 180) return 180;
  return deg;
}

static inline void writeAngle(uint8_t ch, uint8_t deg){
  const uint16_t us = angleToMicros(clamp180(deg));
  pca.setPWM(ch, 0, usToTicks(us));
}
// Call this ONCE in setup/startup
void servosBegin() {
  Wire1.begin();        // safe if already called elsewhere, but fine to have here too
  pca.setOscillatorFrequency(PCA_OSC_HZ); //MAY BREAK THINGS, SHOULD IMPROVE ACCURACY?
  pca.begin();         // wake up PCA9685
  pca.setPWMFreq(SERVO_FREQ_HZ);  // typical analog servo rate ~50 Hz
  delay(10);           // brief settle
  Serial.println("Servos initialized");
}

void move(uint8_t channel, uint8_t angle) {
  writeAngle(channel, angle);


  // if (angle > 180) angle = 180;
  // uint16_t us = SERVO_MIN_US + ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
  // uint16_t offTicks = usToTicks(us);

  // pca.setPWM(channel, 0, offTicks);
}

void setArms(uint8_t Base, uint8_t Shoulder, uint8_t Elbow, uint8_t WristP, uint8_t Claw){
  writeAngle(CH_BASE, Base);
  writeAngle(CH_SHOULDER, Shoulder);
  writeAngle(CH_ELBOW, Elbow);
  writeAngle(CH_WRIST_P, WristP);
  writeAngle(CH_CLAW, Claw);
  Serial.println("angles writen!!!");
}

void servosGoHome(){
  writeAngle(CH_BASE, 90);
  writeAngle(CH_SHOULDER, 90);
  writeAngle(CH_ELBOW, 90);
  writeAngle(CH_WRIST_P, 90);
  writeAngle(CH_CLAW, 30);
}