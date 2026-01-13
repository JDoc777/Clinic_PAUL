#pragma once
#include <Arduino.h>


extern volatile float tin;
extern volatile float hin;
extern volatile float tout;
extern volatile float hout;


// Functions to initialize and read DHT sensors
float getInsideTemp();
float getInsideHumidity();
float getOutsideTemp();
float getOutsideHumidity();
void allDHT(unsigned long min_interval_ms = 2000);
