#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>

#define POT_PIN        A8
#define EMA_ALPHA      0.5

// CHANGE THESE after you measure real raw endpoints
#define POT_RAW_OPEN    3975
#define POT_RAW_CLOSED  435

#define DELTA_TRIP_DEG      25.0f
#define RESIST_COUNT_TRIP   6
#define OPEN_RELEASE_MARGIN 4.0f

const int   ADC_MAX = 4095;
const float V_REF   = 3.3;

struct PotState {
  int   rawValue;
  float smoothedValue;
  float voltage;
  int   percentage;
  float speed;
  float decel;

  float servoAngleDeg;
  float potAngleDeg;
  float deltaDeg;

  int   flag;
};

void      pot_init();
PotState  pot_read();
void      pot_print(const PotState& s);

void      pot_set_claw_cmd(int cmd);

int       pot_get_flag();
void      pot_reset_stall();

#endif