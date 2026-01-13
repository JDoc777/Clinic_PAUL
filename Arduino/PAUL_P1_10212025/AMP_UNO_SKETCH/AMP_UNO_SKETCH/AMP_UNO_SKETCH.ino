#include <Servo.h>

// ==================== GLOBAL OBJECTS ====================
Servo myServo;

// ==================== PIN ASSIGNMENTS ====================
const int SERVO_PIN   = 9;
const int CURRENT_PIN = A1;
const int VOLTAGE_PIN = A0;

// ==================== SENSOR CALIBRATION ====================
// ACS712ELC-05A / -20A → 185 mV per A
const float VREF = 5.0;
const float SENSITIVITY = 0.185;   // V per A
const float DIVIDER_RATIO = 5.0;   // 0–25V voltage module ≈ 5:1

// ==================== EXPECTED RANGES ====================
const float MAX_VOLTAGE = 12.0;
const float MAX_CURRENT = 2.0;

// ==================== SERVO CONTROL ====================
float servoPos = 0;          // global servo position
int stepDir = 1;             // +1 = forward, -1 = backward
const int STEP_SIZE = 3;     // degrees per step
unsigned long moveTimer = 0;
const unsigned long MOVE_INTERVAL = 2;  // ms per step update
const unsigned long FlagLatchTime = 3000;

// ==================== FLAGS / STATES ====================
bool flagSweep = true;       // normal sweeping motion
bool flagAvoid = false;      // resistance detected → avoidance
bool resistanceFlag = false; // raw current-based flag

// ==================== RESISTANCE DETECTION ====================
float avgCurrent = 0;
const float TRIGGER_LEVEL = 15.0;  // normalized %
const int BACKSTEP_HOLD = 120;     // ms pause when switching direction

// ==================== FUNCTION DECLARATIONS ====================
float readCurrentAmps();
float readVoltage();
void stepSweep();
void stepAvoid();
void checkResistance(float normCurrent);
void plotData(float normCurrent, float normVoltage);

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo.write(servoPos);
  delay(1000);
  Serial.println("Servo sweep + avoidance flags active...");
}

// ==================== LOOP ====================
void loop() {
  // ---- Read sensors ----
  float current = readCurrentAmps();
  float voltage = readVoltage();

  avgCurrent = 0.8 * avgCurrent + 0.2 * current;
  float normCurrent = (fabs(avgCurrent) / MAX_CURRENT) * 100.0;
  float normVoltage = (voltage / MAX_VOLTAGE) * 100.0;
  normCurrent = constrain(normCurrent, 0, 100);
  normVoltage = constrain(normVoltage, 0, 100);

  // ---- Check for resistance ----
  checkResistance(normCurrent);

  // ---- Choose motion mode ----
  if (flagAvoid)
    stepAvoid();  // reverse direction until clear
  else
    stepSweep();  // normal sweeping motion

  // ---- Plot for Serial Plotter ----
  plotData(normCurrent, normVoltage);

  delay(2);
}

// ==================== FUNCTIONS ====================

// --- Read current (amps) ---
float readCurrentAmps() {
  int raw = analogRead(CURRENT_PIN);
  float v = raw * (VREF / 1023.0);
  return (v - 2.5) / SENSITIVITY;
}

// --- Read voltage (volts) ---
float readVoltage() {
  int raw = analogRead(VOLTAGE_PIN);
  return raw * (VREF / 1023.0) * DIVIDER_RATIO;
}

// --- Normal sweep motion (0↔180) ---
void stepSweep() {
  if (millis() - moveTimer >= MOVE_INTERVAL) {
    moveTimer = millis();
    servoPos += stepDir * STEP_SIZE;

    // Flip direction at edges
    if (servoPos >= 180) { servoPos = 180; stepDir = -1; }
    if (servoPos <= 0)   { servoPos = 0;   stepDir =  1; }

    myServo.write(servoPos);
  }
}

//--- Avoidance motion (reverse direction while resistance present) ---
//THIS WORKS BUT THE STEPBACK IS TOO FAR
// void stepAvoid() {
//   if (millis() - moveTimer >= MOVE_INTERVAL) {
//     moveTimer = millis();

//     // Move opposite of normal direction to retreat
//     servoPos -= stepDir * (STEP_SIZE * 0.4);

//     // Keep within safe limits
//     servoPos = constrain(servoPos, 0, 180);
//     myServo.write(servoPos);
//   }
// }

//trying this to shorten stepback
void stepAvoid() {
  static int stepsLeft = 0;          // how many back steps to do

  if (stepsLeft == 0) {
    stepsLeft = 1;                   // try 3 backsteps total
  }

  if (millis() - moveTimer >= MOVE_INTERVAL && stepsLeft > 0) {
    moveTimer = millis();

    servoPos -= stepDir * STEP_SIZE; // same speed as normal
    servoPos = constrain(servoPos, 0, 180);
    myServo.write(servoPos);

    stepsLeft--;
  }

  if (!flagAvoid) {
    stepsLeft = 0;                   // reset for next time
  }
}


// --- Check resistance and set flags ---
// void checkResistance(float normCurrent) {
//   if (normCurrent > TRIGGER_LEVEL) {
//     if (!resistanceFlag) {
//       resistanceFlag = true;
//       flagSweep = false;
//       flagAvoid = true;
//       Serial.print("⚠ Resistance detected at ");
//       Serial.print(servoPos);
//       Serial.println("° → switching to AVOID mode");
//       delay(BACKSTEP_HOLD);
//     }
//   } 
//   else {
//     if (resistanceFlag) {
//       resistanceFlag = false;
//       flagSweep = true;
//       flagAvoid = false;
//       Serial.print("✅ Obstacle cleared at ");
//       Serial.print(servoPos);
//       Serial.println("° → resuming SWEEP mode");
//       delay(BACKSTEP_HOLD);
//     }
//   }
// }

// --- Check resistance and set flag once (latching behavior) ---
void checkResistance(float normCurrent) {
  static unsigned long triggerTime = 0;  // time when triggered

  // Trigger once if current exceeds threshold and latch isn't already set
  if (normCurrent > TRIGGER_LEVEL && !resistanceFlag) {
    resistanceFlag = true;
    flagAvoid = !flagAvoid;  // toggle once per trigger
    triggerTime = millis();
    Serial.print("⚠ Triggered latch at ");
    Serial.print(servoPos);
    Serial.print("° → flagAvoid is now ");
    Serial.println(flagAvoid ? "ON" : "OFF");
    delay(BACKSTEP_HOLD);
  }
  if (resistanceFlag && (millis() - triggerTime >= FlagLatchTime)) {
    resistanceFlag = false;
    flagAvoid = !flagAvoid;
    Serial.print("✅ Auto-reset after 1s at ");
    Serial.print(servoPos);
    Serial.println("° → resuming SWEEP mode");
  }
}

// --- Serial Plotter output ---
void plotData(float normCurrent, float normVoltage) {
  Serial.print(normCurrent);                 // Blue = current
  Serial.print(",");
  Serial.print(normVoltage);                 // Orange = voltage
  Serial.print(",");
  Serial.println(flagAvoid ? 100 : 0);       // Green = flag (avoid active)
}
