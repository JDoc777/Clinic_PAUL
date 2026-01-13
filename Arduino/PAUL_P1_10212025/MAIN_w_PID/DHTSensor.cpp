#include "DHTSensor.h"
#include "Setup.h"      // gives access to dht0, dht1 from Setup.cpp
#include <DHT.h>
#include <math.h>

// Use the objects defined in Setup.cpp
extern DHT dht0;
extern DHT dht1;

// Define the globals declared in DHTSensor.h
volatile float tin  = NAN;
volatile float hin  = NAN;
volatile float tout = NAN;
volatile float hout = NAN;

//simple calibration fix
static float tout_offset = -1.4f;
static float tin_offset = -3.0f;
static float hout_offset = +18.6f;
static float hin_offset = +13.6;


float getInsideTemp() {
  tin = dht0.readTemperature();
  if (isnan(tin)){
    tin = 250;
    Serial.println("Error: Failed to read temperature of inside DHT sensor");
  }
  else {
    tin += tin_offset;  // move here
    Serial.print("Inside Temperature = ");
    Serial.println(tin);
    
  }
  return tin; 
}

float getInsideHumidity() {
  hin = dht0.readHumidity();
  if (isnan(hin)){
    hin = 101;
    Serial.println("Error: Failed to read humidity of inside DHT sensor");
  }
  else {
    hin += hin_offset;  // move here
    Serial.print("Inside Himidity = ");
    Serial.println(hin);
  }
  return hin;
}

float getOutsideTemp() {
  tout = dht1.readTemperature();
  if (isnan(tout)){
    tout = 250;
    Serial.println("Error: Failed to read temperature of outside DHT sensor");
  }
  else {
    tout += tout_offset;  // move here
    Serial.print("Outside Temperature = ");
    Serial.println(tout);
  }
  return tout;
}  

float getOutsideHumidity() {
  hout = dht1.readHumidity();
  if (isnan(hout)){
    hout = 101;
    Serial.println("Error: Failed to read humidity of outside DHT sensor");
  }
  else {
    hout += hout_offset;  // move here
    Serial.print("Outside Himidity = ");
    Serial.println(hout);
  }
  return hout;
}

void allDHT(unsigned long min_interval_ms) {
  static unsigned long lastRead = 0;
  unsigned long now = millis();

  if (now - lastRead < min_interval_ms){
    return;
  }

  lastRead = now;

  getInsideTemp();
  getInsideHumidity();
  getOutsideTemp();
  getOutsideHumidity();
}