#pragma once
#include <Arduino.h>


void readEncoder1();
void readEncoder2();
void readEncoder3();
void readEncoder4();

extern volatile long encoder1Count;
extern volatile long encoder2Count;
extern volatile long encoder3Count;
extern volatile long encoder4Count;