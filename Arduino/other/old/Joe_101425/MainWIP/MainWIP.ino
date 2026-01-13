#include "PinDefinitions.h"
#include "DHTSensor.h"
#include "Setup.h"
#include "Sonar.h"
#include "UART.h"
#include "AccelGyro.h"

long sonarDistances[NUM_SONARS];  // storage for all 4 sensors

static UART::Payload p;
float ax, ay, az, gx, gy, gz; //AccelGyro data init

// Example static payload with fake test values
static UART::Payload fakePayload = {
  /* sonar */ 500, 510, 520, 530,
  /* enc   */ 12345, -2345, 989898, -10,
  /* DHTI  */ 55, 3, 22, 8,
  /* DHTO  */ 48, 5, 21, 9,
  /* flags */ 0x0005,            // relays 0 and 2 ON
  /* gyro  */ 1.23f, 2.34f, -0.75f,
  /* accel */ 0.10f, -0.02f, 0.00f
};



void setup() {
  startup();
  settozero();
}



// ---- Build a fresh payload each loop ----
// Uses your existing fakePayload as a template and overwrites enc_* with live posi*
static inline UART::Payload makePayload() {
  UART::Payload pay = fakePayload;   // start with template (default values)

  // Atomic snapshot of encoder posi counters from ISRs
  noInterrupts();
  long p1 = posi1, p2 = posi2, p3 = posi3, p4 = posi4;
  interrupts();

  // Encoder mapping: 1=FR, 2=FL, 3=BL(=RL), 4=BR(=RR)
  pay.enc_FR = (int32_t)p1;
  pay.enc_FL = (int32_t)p2;
  pay.enc_RL = (int32_t)p3;
  pay.enc_RR = (int32_t)p4;

  // Sonar readings
  readAllSonars(sonarDistances);
  pay.sonar_F = (int16_t)sonarDistances[0];  // Front
  pay.sonar_B = (int16_t)sonarDistances[1];  // Back (rear)
  pay.sonar_L = (int16_t)sonarDistances[2];  // Left
  pay.sonar_R = (int16_t)sonarDistances[3];  // Right

  //Accelerometer Readings
  pay.accel_surge = ax;
  pay.accel_sway = ay;
  pay.accel_heave = az;

  // Gyro (deg) Readings
  pay.gyro_roll = gx;
  pay.gyro_pitch = gy;
  pay.gyro_yaw = gz;
 

  return pay;
}




void loop() {

  //DHT function
  allDHT(2000);

  //UART::service(&p, 100);
  if (!IMU::read(ax,ay,az,gx,gy,gz)){
    Serial.print("IMU read unsuccessful");
  }
  // build payload for this cycle
  UART::Payload p_now = makePayload();

  UART::service(&p_now, 10);
  //UART::service(nullptr, 50);   // receive-only, keep parser fed

  noInterrupts();
  long fr = posi1, fl = posi2, bl = posi3, br = posi4;
  interrupts();
  // Print Encoder
  Serial.print("FR: "); Serial.println(posi1);
  Serial.print("FL: ");  Serial.println(posi2);
  Serial.print("RL: "); Serial.println(posi3);
  Serial.print("RR: ");  Serial.println(posi4);


  // // Print Sonar
  // Serial.print("Front: ");  Serial.print(sonarDistances[0]); Serial.print(" cm | ");
  // Serial.print("Back: ");   Serial.print(sonarDistances[1]); Serial.print(" cm | ");
  // Serial.print("Left: ");   Serial.print(sonarDistances[2]); Serial.print(" cm | ");
  // Serial.print("Right: ");  Serial.println(sonarDistances[3]); Serial.print(" cm");

  // // Print Gyro & Accel
  // Serial.println("Surge: "); Serial.print(ax); Serial.print(" G  ");
  // Serial.println("Sway: ");  Serial.print(ay); Serial.print(" G  ");
  // Serial.println("Heave: "); Serial.print(az); Serial.print(" G  ");
  // Serial.println("Roll: ");  Serial.print(gx); Serial.print(" deg/s  ");
  // Serial.println("Pitch: "); Serial.print(gy); Serial.print(" deg/s  ");
  // Serial.println("Yaw: ");   Serial.print(gz); Serial.print(" deg/s");

  // // Delay
  // delay(1000);
}
