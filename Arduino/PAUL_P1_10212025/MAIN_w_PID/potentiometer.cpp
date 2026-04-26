#include "potentiometer.h"
#include <math.h>

static float         smoothedValue = 0;
static float         lastSmoothed  = 0;
static float         lastSpeed     = 0;
static unsigned long lastTime      = 0;

static float servoAngleDegCmd = 180.0f;
static int   latestFlag       = 0;

static float lastServoAngleDeg = 180.0f;
static int   deltaCount        = 0;
static bool  stallLatched      = false;
static float holdClawDeg       = 180.0f;

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (fabs(in_max - in_min) < 1e-6f) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void pot_init() {
  analogReadResolution(12);

  int firstRaw = analogRead(POT_PIN);

  smoothedValue = firstRaw;
  lastSmoothed  = smoothedValue;
  lastSpeed     = 0.0f;
  lastTime      = millis();

  float initialPotAngle = mapFloat(
    smoothedValue,
    (float)POT_RAW_OPEN,
    (float)POT_RAW_CLOSED,
    0.0f,
    180.0f
  );

  if (initialPotAngle < 0.0f)   initialPotAngle = 0.0f;
  if (initialPotAngle > 180.0f) initialPotAngle = 180.0f;

  servoAngleDegCmd  = initialPotAngle;
  latestFlag        = 0;

  lastServoAngleDeg = initialPotAngle;
  deltaCount        = 0;
  stallLatched      = false;
  holdClawDeg       = initialPotAngle;
}

void pot_reset_stall() {
  latestFlag = 0;

  deltaCount   = 0;
  stallLatched = false;
}

int pot_get_flag() {
  return latestFlag;
}

void pot_set_claw_cmd(int cmd) {
  if (cmd < 0)   cmd = 0;
  if (cmd > 180) cmd = 180;

  servoAngleDegCmd = (float)cmd;
}

PotState pot_read() {
  unsigned long now = millis();
  float elapsed = (now - lastTime) / 1000.0f;
  if (elapsed < 0.001f) elapsed = 0.001f;

  int rawValue = analogRead(POT_PIN);

  smoothedValue = EMA_ALPHA * rawValue + (1.0f - EMA_ALPHA) * smoothedValue;

  PotState s;
  s.rawValue      = rawValue;
  s.smoothedValue = smoothedValue;
  s.voltage       = (smoothedValue / ADC_MAX) * V_REF;
  s.percentage    = (int)mapFloat(smoothedValue, 0.0f, (float)ADC_MAX, 0.0f, 100.0f);
  s.speed         = fabs(smoothedValue - lastSmoothed) / elapsed;
  s.decel         = (lastSpeed - s.speed) / elapsed;

  float potAngle = mapFloat(
    smoothedValue,
    (float)POT_RAW_OPEN,
    (float)POT_RAW_CLOSED,
    0.0f,
    180.0f
  );

  if (potAngle < 0.0f)   potAngle = 0.0f;
  if (potAngle > 180.0f) potAngle = 180.0f;

  s.servoAngleDeg = servoAngleDegCmd;
  s.potAngleDeg   = potAngle;
  s.deltaDeg      = s.servoAngleDeg - s.potAngleDeg;

  bool closingCmd = (s.servoAngleDeg > lastServoAngleDeg + 0.5f);

  if (stallLatched) {
    s.flag = 1;
    latestFlag = 1;

    // clear latch once user commands opening
    if (s.servoAngleDeg < holdClawDeg - OPEN_RELEASE_MARGIN) {
      stallLatched = false;
      deltaCount   = 0;
      s.flag       = 0;
      latestFlag   = 0;
    }
  } else {
    if (closingCmd && s.deltaDeg > DELTA_TRIP_DEG) {
      deltaCount++;
    } else {
      deltaCount = 0;
    }

    if (deltaCount >= RESIST_COUNT_TRIP) {
      stallLatched = true;
      holdClawDeg  = s.servoAngleDeg;
      s.flag       = 1;
      latestFlag   = 1;
    } else {
      s.flag = 0;
      latestFlag = 0;
    }
  }

  lastServoAngleDeg = s.servoAngleDeg;

  lastSmoothed = smoothedValue;
  lastSpeed    = s.speed;
  lastTime     = now;

  return s;
}

void pot_print(const PotState& s) {
  Serial.print("Raw: ");
  Serial.print(s.rawValue);

  Serial.print(" | Smoothed: ");
  Serial.print(s.smoothedValue, 1);

  Serial.print(" | Voltage: ");
  Serial.print(s.voltage, 2);

  Serial.print(" | Position: ");
  Serial.print(s.percentage);

  Serial.print(" | ServoAngle: ");
  Serial.print(s.servoAngleDeg, 1);

  Serial.print(" | PotAngle: ");
  Serial.print(s.potAngleDeg, 1);

  Serial.print(" | delta: ");
  Serial.print(s.deltaDeg, 2);

  Serial.print(" | Speed: ");
  Serial.print(s.speed, 1);

  Serial.print(" | Decel: ");
  Serial.print(s.decel, 1);

  Serial.print(" | Flag: ");
  Serial.println(s.flag);
}