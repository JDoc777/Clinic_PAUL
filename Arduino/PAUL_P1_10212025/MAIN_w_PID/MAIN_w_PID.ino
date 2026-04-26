#include "PinDefinitions.h"
#include "DHTSensor.h"
#include "Setup.h"
#include "Sonar.h"
#include "UART.h"
#include "AccelGyro.h"
#include "Buzzer.h"
#include "PCAServo.h"
#include "MeasureCurrent.h"
#include "Motors.h"
#include "potentiometer.h"

static int lastClawSent = 180;

extern volatile long posi1, posi2, posi3, posi4;

long sonarDistances[NUM_SONARS];

int16_t flags777 = 0;

unsigned long lastSample = 0;
unsigned long SAMPLE_PERIOD_US = 100;

// persistent TX payload
static UART::Payload tx_payload;

// IMU state
float ax, ay, az, gx, gy, gz;

// These are defined in Motors.cpp
extern float omegaFR_raw;
extern float omegaFL_raw;
extern float omegaRL_raw;
extern float omegaRR_raw;

extern float omegaFR_f;
extern float omegaFL_f;
extern float omegaRL_f;
extern float omegaRR_f;

// This function is defined in Motors.cpp
extern void updateMotorVelocityEstimate();

static const UART::Payload kTemplate = {
  0, 0, 0, 0,     // sonar
  0, 0, 0, 0,     // enc
  55, 3, 22, 8,   // DHTI
  48, 5, 21, 9,   // DHTO
  (uint16_t)0x0000,
  1.23f, 2.34f, -0.75f,  // gyro
  0.10f, -0.02f, 0.00f   // accel
};

static UART::Payload makePayload() {
  UART::Payload p = kTemplate;

  noInterrupts();
  long p1 = posi1, p2 = posi2, p3 = posi3, p4 = posi4;
  interrupts();

  p.enc_FR = (int32_t)p1;
  p.enc_FL = (int32_t)p2;
  p.enc_RL = (int32_t)p3;
  p.enc_RR = (int32_t)p4;

  readAllSonars(sonarDistances);
  p.sonar_F = sonarDistances[0];
  p.sonar_B = sonarDistances[1];
  p.sonar_L = sonarDistances[2];
  p.sonar_R = sonarDistances[3];

  p.accel_surge = ax;
  p.accel_sway  = ay;
  p.accel_heave = az;

  p.gyro_roll  = gx;
  p.gyro_pitch = gy;
  p.gyro_yaw   = gz;

  uint16_t flags = 0;
  flags |= (flagAvoid << 0);
  p.flags = flags;

  float tin_safe  = isnan(tin)  ? 0.0f : tin;
  float hin_safe  = isnan(hin)  ? 0.0f : hin;
  float tout_safe = isnan(tout) ? 0.0f : tout;
  float hout_safe = isnan(hout) ? 0.0f : hout;

  p.dhti_h    = (uint8_t)hin_safe;
  p.dhti_hdec = (uint8_t)((hin_safe - p.dhti_h) * 10.0f);

  p.dhti_t    = (uint8_t)tin_safe;
  p.dhti_tdec = (uint8_t)((tin_safe - p.dhti_t) * 10.0f);

  p.dhto_h    = (uint8_t)hout_safe;
  p.dhto_hdec = (uint8_t)((hout_safe - p.dhto_h) * 10.0f);

  p.dhto_t    = (uint8_t)tout_safe;
  p.dhto_tdec = (uint8_t)((tout_safe - p.dhto_t) * 10.0f);

  return p;
}
void safeSetArms(int base, int shoulder, int elbow, int wrist, int claw) {
  base     = constrain(base, 0, 180);
  shoulder = constrain(shoulder, 0, 180);
  elbow    = constrain(elbow, 0, 180);
  wrist    = constrain(wrist, 0, 180);
  claw     = constrain(claw, 0, 180);

  // if claw is opening, clear stall
  if (claw < lastClawSent - 1) {
    pot_reset_stall();
  }

  // if stalled, block more closing
  if (pot_get_flag() == 1 && claw > lastClawSent) {
    claw = lastClawSent;
  }

  pot_set_claw_cmd(claw);
  setArms(base, shoulder, elbow, wrist, claw);

  lastClawSent = claw;
}

void setup() {
  startup();
  servosGoHome();
  pot_init();
  initMotorPID(7.0f, 4.0f, 0.0f);
}

void loop() {

  PotState s = pot_read();


  ampFlag();

  allDHT(2000);

  if (!IMU::read(ax, ay, az, gx, gy, gz)) {
    // IMU failed
  }

  // Update wheel velocity estimates in Motors.cpp
  updateMotorVelocityEstimate();

  UART::service();

  tx_payload = makePayload();


  UART::sendTelemetry(tx_payload);

  unsigned long now = micros();

  //applyPWM(200, 1, 1, 1


  if (now - lastSample >= SAMPLE_PERIOD_US) {
    lastSample = now;

    //Serial.print(now);
    //Serial.print(",");

    // Keep same CSV order as before:
    // omega1, omega2, omega3, omega4, omega1_f, omega2_f, omega3_f, omega4_f
    // where 1=FR, 2=FL, 3=RL, 4=RR
    /*Serial.print(omegaFR_raw); Serial.print(",");
    Serial.print(omegaFL_raw); Serial.print(",");
    Serial.print(omegaRL_raw); Serial.print(",");
    Serial.print(omegaRR_raw); Serial.print(",");*/

    Serial.print(omegaFR_f); Serial.print(",");
    Serial.print(omegaFL_f); Serial.print(",");
    Serial.print(omegaRL_f); Serial.print(",");
    Serial.println(omegaRR_f);
  }
}