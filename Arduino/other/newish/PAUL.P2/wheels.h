#pragma once
#include "setup_paul.h"

// ====== Tuning ======
#ifndef PULSES_PER_REV
#define PULSES_PER_REV 600   // <-- put your real CPR here
#endif

// Optional per-wheel direction flip (1 = invert, 0 = normal)
#ifndef ENC_INV_FR
#define ENC_INV_FR 1
#endif
#ifndef ENC_INV_FL
#define ENC_INV_FL 0   // you said FL counts the wrong way; start inverted
#endif
#ifndef ENC_INV_BR
#define ENC_INV_BR 1
#endif
#ifndef ENC_INV_BL
#define ENC_INV_BL 0
#endif

// Public counters (signed)
extern volatile long posiFR;
extern volatile long posiFL;
extern volatile long posiBR;
extern volatile long posiBL;

// Rotations (floating; updated in ISRs for convenience)
extern volatile float rotationsFR;
extern volatile float rotationsFL;
extern volatile float rotationsBR;
extern volatile float rotationsBL;

// Setup / utility
void encodersBegin();         // sets pinModes and attachInterrupts
void encodersReset();         // zero all counts/rotations
void encodersPrintCounts();   // Serial print FR/FL/BR/BL
