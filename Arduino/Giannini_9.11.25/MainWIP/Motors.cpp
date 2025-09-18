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
