#include "Buzzer.h"
#include "PinDefinitions.h"

unsigned lastHz = 0;  // global or static

void playFromBytes(uint8_t k_khz, uint8_t k_hundredths) {
  unsigned hz = static_cast<unsigned>(k_khz) * 1000u
              + static_cast<unsigned>(k_hundredths) * 10u;

  // Stop if 0
  if (hz == 0u) {
    if (lastHz != 0) {
      noTone(BUZZER);
      lastHz = 0;
    }
    return;
  }

  // Clamp to 2–4 kHz range
  if (hz < 2000u) hz = 2000u;
  if (hz > 4000u) hz = 4000u;

  // Only update if frequency changed
  if (hz != lastHz) {
    tone(BUZZER, hz);
    lastHz = hz;
  }
}
