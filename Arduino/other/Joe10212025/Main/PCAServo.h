
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


void servosBegin();

void setArms(uint8_t Base, uint8_t Shoulder, uint8_t Elbow, uint8_t WristP, uint8_t Claw);

void move(uint8_t channel, uint8_t angle);