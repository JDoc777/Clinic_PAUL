#include "PotMonitor.h"

PotMonitor::PotMonitor(int pin, int thresh, unsigned long stable_ms) {
    potPin = pin;
    threshold = thresh;
    stableTime = stable_ms;

    potValue = 0;
    lastPotValue = 0;
    voltage = 0.0;
    potStable = true;
    lastChangeTime = 0;
}

void PotMonitor::begin() {
    pinMode(potPin, INPUT);

    potValue = analogRead(potPin);
    lastPotValue = potValue;
    voltage = potValue * (5.0 / 1023.0);
    potStable = true;
    lastChangeTime = millis();
}

void PotMonitor::update() {
    potValue = analogRead(potPin);
    voltage = potValue * (5.0 / 1023.0);

    if (abs(potValue - lastPotValue) > threshold) {
        potStable = false;
        lastChangeTime = millis();
    }

    if (millis() - lastChangeTime > stableTime) {
        potStable = true;
    }

    lastPotValue = potValue;

    // Serial Monitor + Serial Plotter friendly output
    Serial.print("pot:");
    Serial.print(potValue);
    Serial.print("\tvoltage:");
    Serial.print(voltage, 3);
    Serial.print("\tstable:");
    Serial.println(potStable ? 1 : 0);
}
int PotMonitor::getPotValue() {
    return potValue;
}

float PotMonitor::getVoltage() {
    return voltage;
}

bool PotMonitor::isStable() {
    return potStable;
}