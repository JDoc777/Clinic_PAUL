#ifndef POTMONITOR_H
#define POTMONITOR_H

#include <Arduino.h>

class PotMonitor {
private:
    int potPin;
    int potValue;
    int lastPotValue;
    float voltage;
    bool potStable;

    int threshold;
    unsigned long stableTime;
    unsigned long lastChangeTime;

public:
    PotMonitor(int pin, int thresh = 3, unsigned long stable_ms = 200);

    void begin();
    void update();

    int getPotValue();
    float getVoltage();
    bool isStable();
};

#endif