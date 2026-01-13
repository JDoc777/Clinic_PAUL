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

long sonarDistances[NUM_SONARS];

int16_t flags777 = 0;

// persistent TX payload
static UART::Payload tx_payload;

// IMU state
float ax, ay, az, gx, gy, gz;

static const UART::Payload kTemplate = {
  /* sonar */ 0, 0, 0, 0,
  /* enc   */ 0, 0, 0, 0,
  /* DHTI  */ 55, 3, 22, 8,
  /* DHTO  */ 48, 5, 21, 9,
  /* flags */ (uint16_t)0x0000,
  /* gyro  */ 1.23f, 2.34f, -0.75f,
  /* accel */ 0.10f, -0.02f, 0.00f
};

static UART::Payload makePayload() {
  UART::Payload p = kTemplate;

  // atomic snapshot of encoders
  noInterrupts();
  long p1 = posi1, p2 = posi2, p3 = posi3, p4 = posi4;
  interrupts();

  p.enc_FR = (int32_t)p1;
  p.enc_FL = (int32_t)p2;
  p.enc_RL = (int32_t)p3;
  p.enc_RR = (int32_t)p4;

  // sonar readings
  readAllSonars(sonarDistances);
  p.sonar_F = (int16_t)sonarDistances[0];
  p.sonar_B = (int16_t)sonarDistances[1];
  p.sonar_L = (int16_t)sonarDistances[2];
  p.sonar_R = (int16_t)sonarDistances[3];

  // IMU
  p.accel_surge = ax;
  p.accel_sway  = ay;
  p.accel_heave = az;

  p.gyro_roll  = gx;
  p.gyro_pitch = gy;
  p.gyro_yaw   = gz;

  Serial.print("Flag avoid: ");
  Serial.println(flagAvoid);

  uint16_t flags777 = 0;          // reset each payload
  flags777 |= (flagAvoid << 0);   // bit 0 = pincher current flag
  p.flags = flags777;              // pack into payload

  Serial.print("payload flag: ");
  Serial.println(flags777);

  float tin_safe  = isnan(tin)  ? 0.0f : tin;
  float hin_safe  = isnan(hin)  ? 0.0f : hin;
  float tout_safe = isnan(tout) ? 0.0f : tout;
  float hout_safe = isnan(hout) ? 0.0f : hout;

  // Inside humidity
  p.dhti_h    = (uint8_t)hin_safe;
  p.dhti_hdec = (uint8_t)((hin_safe - p.dhti_h) * 10.0f);

  // Inside temp
  p.dhti_t    = (uint8_t)tin_safe;
  p.dhti_tdec = (uint8_t)((tin_safe - p.dhti_t) * 10.0f);

  // Outside humidity
  p.dhto_h    = (uint8_t)hout_safe;
  p.dhto_hdec = (uint8_t)((hout_safe - p.dhto_h) * 10.0f);

  // Outside temp
  p.dhto_t    = (uint8_t)tout_safe;
  p.dhto_tdec = (uint8_t)((tout_safe - p.dhto_t) * 10.0f);

  return p;
}

void setup() {
  startup();
  settozero();

  initMotorPID(
    4.0f,   // kp (tune!)
    0.5f,    // ki (start at 0)
    0.05f     // kd (tune!)
  );

  // Serial.begin(115200);
  // Serial.println("System Init Complete");
}

void loop() {
 
  //Amp Sensor
  ampFlag();

  // sensors
  allDHT(2000);
  if (!IMU::read(ax, ay, az, gx, gy, gz)) {
    Serial.println("IMU read unsuccessful");
  }




  // setArms(90,90,90,90,90);
  // delay(2);
  // setArms(180,180,180,180,180);
  // delay(2);
  // Build TX payload
  tx_payload = makePayload();

  // Always service RX
  UART::service();
  //digitalWrite(Relay_motors, HIGH);


  // Always send TX
  UART::sendTelemetry(tx_payload);

  //Print Sonar out
  Serial.print("Front: ");  Serial.print(sonarDistances[0]); Serial.print(" cm | ");
  Serial.print("Back: ");   Serial.print(sonarDistances[1]); Serial.print(" cm | ");
  Serial.print("Left: ");   Serial.print(sonarDistances[2]); Serial.print(" cm | ");
  Serial.print("Right: ");  Serial.print(sonarDistances[3]); Serial.println(" cm");
}