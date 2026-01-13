#include "PinDefinitions.h"
#include "MeasureCurrent.h"

// ==================== DEFINITIONS ====================
const float VREF = 5.0;
const float SENSITIVITY = 0.185;   // V per A
const float MAX_CURRENT = 2.0;

bool flagAvoid = false;
bool resistanceFlag = false;

float avgCurrent = 0;
const float TRIGGER_LEVEL = 400.0;
const unsigned long FlagLatchTime = 1000;  // 1 second

void ampFlag(){
  float current = readCurrentAmps();

  avgCurrent = 0.8 * avgCurrent + 0.2 * current;
  float normCurrent = (fabs(avgCurrent) / MAX_CURRENT) * 100.0;

  checkResistance(normCurrent);
}


float readCurrentAmps() {
  int raw = analogRead(CURRENT_PIN);
  float v = raw * (VREF / 1023.0);
  return (v - 2.5) / SENSITIVITY;
}

void checkResistance(float normCurrent) {
  static unsigned long triggerTime = 0;  // time when triggered
  Serial.println(normCurrent);
  Serial.println("!!!!!!!!!!!!!!!!!!!");
  if (normCurrent < TRIGGER_LEVEL && !resistanceFlag) {
    resistanceFlag = true;
    flagAvoid = !flagAvoid;  // toggle once per trigger
    Serial.println(flagAvoid);
    triggerTime = millis();
    Serial.println("⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠⚠ Triggered latch");
  }
  if (resistanceFlag && (millis() - triggerTime >= FlagLatchTime)) {
    resistanceFlag = false;
    flagAvoid = !flagAvoid;
    Serial.println(flagAvoid);
    // Serial.println("✅ Auto-reset after 1s");
  }
}

void plotCurrentRaw() {
  float current = readCurrentAmps();     // read in Amps
  float v = (current * SENSITIVITY) + 2.5;  // back-calc voltage for reference (optional)

  Serial.print("Current_A:"); 
  Serial.print(current, 4);   // plot current in amps (4 decimal places)
  Serial.print("\tVoltage_V:"); 
  Serial.println(v, 3);       // plot voltage for zero calibration visualization
}