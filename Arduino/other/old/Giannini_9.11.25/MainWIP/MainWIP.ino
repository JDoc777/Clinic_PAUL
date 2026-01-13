#include "PinDefinitions.h"
#include "DHT.h"
#include "Setup.h"
#include "Sonar.h"
#include "UART.h"
#include "DHTSensor.h"

long sonarDistances[NUM_SONARS];  // storage for all 4 sensors

static UART::Payload p;

// Example static payload with fake test values
static UART::Payload fakePayload = {
  /* sonar */ 500, 510, 520, 530,
  /* enc   */ 12345, -2345, 989898, -10,
  /* DHTI  */ 55, 3, 22, 8,
  /* DHTO  */ 48, 5, 21, 9,
  /* relay */ 0x0005,            // relays 0 and 2 ON
  /* gyro  */ 1.23f, 2.34f, -0.75f,
  /* accel */ 0.10f, -0.02f, 0.00f
};



void setup() {
  startup();
  settozero();

}

void loop() {

  //UART::service(&p, 100);

  //UART::service(&fakePayload, 100);


  /*DHTData();
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  delay(3000);*/

  /*readAllSonars(sonarDistances);
  Serial.print("Sonar distances: ");
  for (int i = 0; i < NUM_SONARS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(sonarDistances[i]);
    Serial.print(" cm  ");
  }*/





  // // put your main code here, to run repeatedly:
  // Serial.print("Encoder 1 Count: ");
  // Serial.println(posi1);

  // Serial.print("Encoder 2 Count: ");
  // Serial.println(posi2);

  // Serial.print("Encoder 3 Count: ");
  // Serial.println(posi3);

  // Serial.print("Encoder 4 Count: ");
  // Serial.println(posi4);

  // readAllSonars(sonarDistances);

  // // Print them out
  // Serial.print("Front: ");  Serial.print(sonarDistances[0]); Serial.print(" cm | ");
  // Serial.print("Back: ");   Serial.print(sonarDistances[1]); Serial.print(" cm | ");
  // Serial.print("Left: ");   Serial.print(sonarDistances[2]); Serial.print(" cm | ");
  // Serial.print("Right: ");  Serial.println(sonarDistances[3]); Serial.print(" cm");

  // delay(1000);
}

void collectSensorData(){

  readAllSonars(sonarDistances);

}
