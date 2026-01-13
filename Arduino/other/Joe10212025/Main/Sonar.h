#pragma once
#include <Arduino.h>

// Match the count in PinDefinitions.h
constexpr int NUM_SONARS = 4;

// sonar max and min ranges (in cm)
constexpr int SONAR_MAX = 400;
constexpr int SONAR_MIN = 2;
constexpr int MIN_ERROR = 500;
constexpr int MAX_ERROR = 1000;

// Read distance from a single sonar (cm)
long readSonar(int index);

// Read all sonar distances into an array
void readAllSonars(long distances[NUM_SONARS]);

