#include "DHTSensor.h"
#include "Setup.h"       // gives access to dht0, dht1 from Setup.cpp
#include <DHT.h>

// Use the objects defined in Setup.cpp
extern DHT dht0;
extern DHT dht1;

float getInsideTemp()     { return dht0.readTemperature(); }
float getInsideHumidity() { return dht0.readHumidity(); }
float getOutsideTemp()    { return dht1.readTemperature(); }
float getOutsideHumidity(){ return dht1.readHumidity(); }