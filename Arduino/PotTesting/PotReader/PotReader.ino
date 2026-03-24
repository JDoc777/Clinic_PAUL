#include <Servo.h>
#include "PotMonitor.h"

PotMonitor pot(A1);
Servo myServo;

const int servoPin = 9;

// smoothing settings
const int numSamples = 10;
int samples[numSamples];
int sampleIndex = 0;
long sampleTotal = 0;
int smoothedPotValue = 0;

int lastAngle = -1;

void runPotMonitor() {
    pot.update();
}

void setup() {
    Serial.begin(9600);
    pot.begin();

    myServo.attach(servoPin, 500, 2500);
    myServo.write(90);
    delay(1000);

    int initialValue = pot.getPotValue();
    for (int i = 0; i < numSamples; i++) {
        samples[i] = initialValue;
        sampleTotal += samples[i];
    }
    smoothedPotValue = initialValue;
}

void loop() {
    runPotMonitor();

    int rawPotValue = pot.getPotValue();

    // moving average filter
    sampleTotal -= samples[sampleIndex];
    samples[sampleIndex] = rawPotValue;
    sampleTotal += samples[sampleIndex];
    sampleIndex = (sampleIndex + 1) % numSamples;

    smoothedPotValue = sampleTotal / numSamples;

    int angle = map(smoothedPotValue, 0, 1023, 0, 180);

    // only update servo if angle changes enough
    if (abs(angle - lastAngle) >= 2) {
        myServo.write(angle);
        lastAngle = angle;
    }

    Serial.print("\traw:");
    Serial.print(rawPotValue);
    Serial.print("\tsmoothed:");
    Serial.print(smoothedPotValue);
    Serial.print("\tangle:");
    Serial.println(angle);

    delay(20);
}