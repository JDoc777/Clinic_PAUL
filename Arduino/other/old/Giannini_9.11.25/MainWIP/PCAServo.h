#pragma once
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "PinDefinitions.h"


//servos min and max turning
#define ServoMin 0
#define ServoMax 180

namespace PCAServo{
  //set freq
  void begin(int PWMHz = 60);

  //set angles
  void setAngle(int servoCount, int angleDegree);

  //smooth servos movement
  void moveSmooth(int servoCount, int targetDegree, int step = 1, stepDelay = 10);

  //set default positions
  void defaultAngles();
  


}
