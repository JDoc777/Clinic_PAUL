#pragma once
#include <Arduino.h>

// Match the count in PinDefinitions.h
constexpr int NUM_SONARS = 4;

// Read distance from a single sonar (cm)
long readSonar(int index);

// Read all sonar distances into an array
void readAllSonars(long distances[NUM_SONARS]);

