#pragma once
#include <Arduino.h>

void DHT_Init();
bool DHT_Read(float& humidity, float& tempC);
extern volatile float temperature;
extern volatile float humidity;
void DHTData();
/*
// Functions to initialize and read DHT sensors
float getInsideTemp();
float getInsideHumidity();
float getOutsideTemp();
float getOutsideHumidity();*/





