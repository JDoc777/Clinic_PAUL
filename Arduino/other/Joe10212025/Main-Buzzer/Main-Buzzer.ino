#include "PinDefinitions.h"
#include "DHTSensor.h"
#include "Setup.h"
#include "Sonar.h"
#include "UART.h"
#include "AccelGyro.h"
#include "Buzzer.h"

long sonarDistances[NUM_SONARS];

// persistent TX payload
static UART::Payload tx_payload;

// IMU state
float ax, ay, az, gx, gy, gz;

static const UART::Payload kTemplate = {
  /* sonar */ 0, 0, 0, 0,
  /* enc   */ 0, 0, 0, 0,
  /* DHTI  */ 55, 3, 22, 8,
  /* DHTO  */ 48, 5, 21, 9,
  /* flags */ (uint16_t)0x0005,
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

  return p;
}

void setup() {
  startup();
  settozero();

  Serial.begin(115200);
  Serial.println("System Init Complete");
}

void loop() {
  // sensors
  allDHT(2000);
  if (!IMU::read(ax, ay, az, gx, gy, gz)) {
    Serial.println("IMU read unsuccessful");
  }

  // Build TX payload
  tx_payload = makePayload();

  // Always service RX
  UART::service();

  // Always send TX
  UART::sendTelemetry(tx_payload);

  playFromBytes(2, 45);  // plays 2.45 kHz
  //playFromBytes(0, 0);   // stops the buzzer

  // Debug encoders
  Serial.print("FR: "); Serial.println(posi1);
  Serial.print("FL: "); Serial.println(posi2);
  Serial.print("RL: "); Serial.println(posi3);
  Serial.print("RR: "); Serial.println(posi4);

  // Debug sonar
  Serial.print("Front: ");  Serial.print(sonarDistances[0]); Serial.print(" cm | ");
  Serial.print("Back: ");   Serial.print(sonarDistances[1]); Serial.print(" cm | ");
  Serial.print("Left: ");   Serial.print(sonarDistances[2]); Serial.print(" cm | ");
  Serial.print("Right: ");  Serial.println(sonarDistances[3]);
}