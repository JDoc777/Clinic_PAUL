#include "Sonar.h"
#include "PinDefinitions.h"

// Measure distance from one sonar in cm
long readSonar(int index) {
  if (index < 0 || index >= NUM_SONARS) return -1;  // invalid index

  // Trigger pulse
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // Echo pulse duration
  long duration = pulseIn(echoPins[index], HIGH, 30000); // timeout = 30 ms
  // Convert to cm (speed of sound 343 m/s)
  long distance = duration * 0.034 / 2;
  
  if (distance < SONAR_MIN) {
    distance = MIN_ERROR;
  }
  if (distance > SONAR_MAX) {
    distance = MAX_ERROR;
  }

  return distance;
}

// Read all sonars at once
void readAllSonars(long distances[NUM_SONARS]) {
  for (int i = 0; i < NUM_SONARS; i++) {
    distances[i] = readSonar(i);
  }
}
