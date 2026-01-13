#pragma once
#include <Arduino.h>

// Functions to initialize and read DHT sensors
float getInsideTemp();
float getInsideHumidity();
float getOutsideTemp();
float getOutsideHumidity();