#include "wheels.h"

// ====== State ======
volatile long  posiFR = 0, posiFL = 0, posiBR = 0, posiBL = 0;
volatile float rotationsFR = 0, rotationsFL = 0, rotationsBR = 0, rotationsBL = 0;

// ---- Core ISR step: ++ if B is HIGH, -- if LOW (with optional inversion) ----
static inline void stepCount(volatile long &posi, volatile float &rots, int bHigh, bool invert) {
  if (!invert) {
    if (bHigh) posi++; else posi--;
  } else {
    if (bHigh) posi--; else posi++;
  }
  rots = (float)posi / (float)PULSES_PER_REV;
}

// ====== ISRs (trigger on channel A RISING) ======
void ISR_FR_A() { stepCount(posiFR, rotationsFR, digitalRead(ENC_FR_B) == HIGH, ENC_INV_FR); }
void ISR_FL_A() { stepCount(posiFL, rotationsFL, digitalRead(ENC_FL_B) == HIGH, ENC_INV_FL); }
void ISR_BR_A() { stepCount(posiBR, rotationsBR, digitalRead(ENC_BR_B) == HIGH, ENC_INV_BR); }
void ISR_BL_A() { stepCount(posiBL, rotationsBL, digitalRead(ENC_BL_B) == HIGH, ENC_INV_BL); }

void encodersBegin() {
  // Pin modes (you already set these elsewhere as INPUT_PULLUP, but safe to set again)
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP);
  pinMode(ENC_BR_B, INPUT_PULLUP);
  pinMode(ENC_BL_A, INPUT_PULLUP);
  pinMode(ENC_BL_B, INPUT_PULLUP);

  // Attach interrupts on channel A, RISING edge
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), ISR_FR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), ISR_FL_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), ISR_BR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), ISR_BL_A, RISING);

  encodersReset();
}

void encodersReset() {
  noInterrupts();
  posiFR = posiFL = posiBR = posiBL = 0;
  rotationsFR = rotationsFL = rotationsBR = rotationsBL = 0.0f;
  interrupts();
}

void encodersPrintCounts() {
  long fr, fl, br, bl;
  noInterrupts();
  fr = posiFR; fl = posiFL; br = posiBR; bl = posiBL;
  interrupts();

  Serial.print("FR: "); Serial.print(fr);
  Serial.print(" | FL: "); Serial.print(fl);
  Serial.print(" | BR: "); Serial.print(br);
  Serial.print(" | BL: "); Serial.println(bl);
}
