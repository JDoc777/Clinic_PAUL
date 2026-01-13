#include <Arduino.h>
#include "PinDefinitions.h"
#include "Motors.h"

void stopMotors() {
  analogWrite(ENA_FL, 0);
  analogWrite(ENB_FR, 0);
  analogWrite(ENA_BR, 0);
  analogWrite(ENB_BL, 0);

  digitalWrite(IN1_FL, LOW);
  digitalWrite(IN2_FL, LOW);
  digitalWrite(IN3_FR, LOW);
  digitalWrite(IN4_FR, LOW);
  digitalWrite(IN5_BR, LOW);
  digitalWrite(IN6_BR, LOW);
  digitalWrite(IN7_BL, LOW);
  digitalWrite(IN8_BL, LOW);
}

// Function to drive Mecanum wheels
// fl, fr, rl, rr: Speed for each wheel (-255 to 255)
void mecanumDrive(int fl, int fr, int rl, int rr) {
  currentSpeedFL = fl;
  currentSpeedFR = fr;
  currentSpeedRL = rl;
  currentSpeedRR = rr;

  driveMotor(ENA_FL, IN1_FL, IN2_FL, fl); // Front Left
  driveMotor(ENB_FR, IN3_FR, IN4_FR, fr); // Front Right
  driveMotor(ENB_BL, IN7_BL, IN8_BL, rl); // Rear Left
  driveMotor(ENA_BR, IN5_BR, IN6_BR, rr); // Rear Right
}

// Function to control an individual motor
// enablePin: PWM pin, inPin1/inPin2: Direction pins, speed: -255 to 255
void driveMotor(int enablePin, int inPin1, int inPin2, int speed) {
  if (speed > 0) {
    digitalWrite(inPin1, HIGH);       // Forward for (+)
    digitalWrite(inPin2, LOW);
  } else if (speed < 0) {
    digitalWrite(inPin1, LOW);        // Backward for (-)
    digitalWrite(inPin2, HIGH);
    speed = -speed;                   // Make speed positive for PWM
  } else {
    digitalWrite(inPin1, LOW);        // Off for (0)
    digitalWrite(inPin2, LOW);
  }

  analogWrite(enablePin, constrain(speed, 0, 255)); // Set motor speed
}
