#include <Servo.h>
#include "PotMonitor.h"

PotMonitor pot(A1, 5, 400);
Servo myServo;

const int servoPin = 9;
const int openAngle = 20;
const int closeAngle = 160;

const int backoffDegrees = 20;   // 🔥 how far to back off

bool closeCommandSent = false;
bool movementStarted = false;
bool resistanceFlag = false;
bool finished = false;

int lastCommandedAngle = closeAngle;   // track last position

void setup() {
  Serial.begin(115200);
  pot.begin();

  myServo.attach(servoPin, 500, 2500);

  myServo.write(openAngle);
  delay(1500);

  // send ONE close command
  myServo.write(closeAngle);
  lastCommandedAngle = closeAngle;
  closeCommandSent = true;
}

void loop() {
  if (finished) return;

  pot.update();

  // detect real movement
  if (closeCommandSent && !pot.isStable()) {
    movementStarted = true;
  }

  // detect resistance
  if (closeCommandSent && movementStarted && pot.isStable() && !resistanceFlag) {
    resistanceFlag = true;

    // 🔥 BACK OFF A FEW DEGREES
    int backoffAngle = lastCommandedAngle - backoffDegrees;
    if (backoffAngle < openAngle) backoffAngle = openAngle;

    myServo.write(backoffAngle);

    finished = true;

    Serial.println("RESISTANCE -> BACKOFF");
  }

  Serial.print("pot:");
  Serial.print(pot.getPotValue());
  Serial.print("\tstable:");
  Serial.print(pot.isStable() ? 1 : 0);
  Serial.print("\tmoved:");
  Serial.print(movementStarted ? 1 : 0);
  Serial.print("\tflag:");
  Serial.println(resistanceFlag ? 1 : 0);

  delay(20);
}