
#include "PCAServo.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);


// Call this ONCE in setup/startup
void servosBegin() {
  Wire.begin();        // safe if already called elsewhere, but fine to have here too
  pca.begin();         // wake up PCA9685
  pca.setPWMFreq(50);  // typical analog servo rate ~50 Hz
  delay(10);           // brief settle
}

constexpr uint16_t SERVO_MIN_US = 650;   // fully left
constexpr uint16_t SERVO_MAX_US = 2350;  // fully right
constexpr float SERVO_FREQ_HZ   = 50.0f; // standard servo frequency

uint16_t usToTicks(uint16_t us){
  float ticks = (float)us * SERVO_FREQ_HZ * 4096.0f / 1000000.0f;
  return (uint16_t)(ticks + 0.5f);
}

void move(uint8_t channel, uint8_t angle) {
  if (angle > 180) angle = 180;
  uint16_t us = SERVO_MIN_US + ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
  uint16_t offTicks = usToTicks(us);

  pca.setPWM(channel, 0, offTicks);
}

void setArms(uint8_t Base, uint8_t Shoulder, uint8_t Elbow, uint8_t WristP, uint8_t Claw){
  move(15, Base);
  move(14, Shoulder);
  move(13, Elbow);
  move(12, WristP);
  move(11, Claw);
}


