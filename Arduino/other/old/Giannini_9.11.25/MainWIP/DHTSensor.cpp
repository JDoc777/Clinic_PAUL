#include "DHTSensor.h"
#include "PinDefinitions.h"
#include "Setup.h"       // gives access to dht0, dht1 from Setup.cpp
#include <DHT.h>

// Keep the sensor instance private to this file (no externs needed)
static DHT dht0(DHT0_Pin, DHT0_Type);

void DHT_Init() {
  dht0.begin();
}


void DHTData() {
  temperature = dht0.readTemperature();      // Read Temp.
  humidity = dht0.readHumidity();            // Read Humid.
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Error reading temperature or humidity");
    temperature = 25.0;  // Default to 25°C if DHT fails
    humidity = 50.0;     // Default to 50% if DHT fails
  }

}
/*
// Use the objects defined in Setup.cpp
extern DHT dht0;
extern DHT dht1;

float getInsideTemp()     { return dht0.readTemperature(); }
float getInsideHumidity() { return dht0.readHumidity(); }
float getOutsideTemp()    { return dht1.readTemperature(); }
float getOutsideHumidity(){ return dht1.readHumidity(); }*/