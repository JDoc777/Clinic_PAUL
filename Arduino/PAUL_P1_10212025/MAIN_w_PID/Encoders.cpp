#include <Arduino.h>
#include "Encoders.h"
#include "PinDefinitions.h"

// Bring in the counters that are DEFINED in Setup.cpp
extern volatile long posi1, posi2, posi3, posi4;
extern volatile long rotations1, rotations2, rotations3, rotations4;

// Set this to YOUR encoder's pulses per mechanical revolution (per-channel 1×)
inline constexpr long PULSES_PER_REV = 222.5;  // TODO: adjust to your hardware

// ---- 1× decoding: ISR on A (RISING), read B for direction ----
// Mapping assumed: 1=FR, 2=FL, 3=BL, 4=BR
// If a wheel counts backwards, swap ++ and -- in that wheel's ISR.

// void readEncoder1() {                  // FRONT RIGHT (reversed)
//   int b = digitalRead(ENCB_FR);
//   if (b) posi1--; else posi1++;   // <-- flipped
//   if (posi1 >=  PULSES_PER_REV) { posi1 -= PULSES_PER_REV; rotations1++; }
//   else if (posi1 <= -PULSES_PER_REV) { posi1 += PULSES_PER_REV; rotations1--; }
// }

// void readEncoder2() {                  // FRONT LEFT (good)
//   int b = digitalRead(ENCB_FL);
//   if (b) posi2++; else posi2--;
//   if (posi2 >=  PULSES_PER_REV) { posi2 -= PULSES_PER_REV; rotations2++; }
//   else if (posi2 <= -PULSES_PER_REV) { posi2 += PULSES_PER_REV; rotations2--; }
// }

// void readEncoder3() {                  // BACK LEFT (no counts yet; keep as-is for now)
//   int b = digitalRead(ENCB_BL);
//   if (b) posi3++; else posi3--;
//   if (posi3 >=  PULSES_PER_REV) { posi3 -= PULSES_PER_REV; rotations3++; }
//   else if (posi3 <= -PULSES_PER_REV) { posi3 += PULSES_PER_REV; rotations3--; }
// }

// void readEncoder4() {                  // BACK RIGHT (reversed)
//   int b = digitalRead(ENCB_BR);
//   if (b) posi4--; else posi4++;   // <-- flipped
//   if (posi4 >=  PULSES_PER_REV) { posi4 -= PULSES_PER_REV; rotations4++; }
//   else if (posi4 <= -PULSES_PER_REV) { posi4 += PULSES_PER_REV; rotations4--; }
// }



//JUSTINS ENCODER CODE
void readEncoder1() {

  int b = digitalRead(ENCB_FR);

  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }

  if (posi1 >= PULSES_PER_REV) {
    posi1 -= PULSES_PER_REV;
    rotations1++;
  } else if (posi1 <= -PULSES_PER_REV) {
    posi1 += PULSES_PER_REV;
    rotations1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCB_FL);

  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }

  if (posi2 >= PULSES_PER_REV) {
    posi2 -= PULSES_PER_REV;
    rotations2++;
  } else if (posi2 <= -PULSES_PER_REV) {
    posi2 += PULSES_PER_REV;
    rotations2--;
  }
}

void readEncoder3() {
  int b = digitalRead(ENCB_BL);

  if (b > 0) {
    posi3++;
  } else {
    posi3--;
  }

  if (posi3 >= PULSES_PER_REV) {
    posi3 -= PULSES_PER_REV;
    rotations3++;
  } else if (posi3 <= -PULSES_PER_REV) {
    posi3 += PULSES_PER_REV;
    rotations3--;
  }
}

void readEncoder4() {
  int b = digitalRead(ENCB_BR);

  if (b > 0) {
    posi4++;
  } else {
    posi4--;
  }

  if (posi4 >= PULSES_PER_REV) {
    posi4 -= PULSES_PER_REV;
    rotations4++;
  } else if (posi4 <= -PULSES_PER_REV) {
    posi4 += PULSES_PER_REV;
    rotations4--;
  }
}