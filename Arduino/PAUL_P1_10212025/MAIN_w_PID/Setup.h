#pragma once
void startup();
void settozero();

//extern volatile uint16_t flags777 = 0;

// Expose encoder state to other translation units:
extern volatile long posi1, posi2, posi3, posi4;
extern volatile long rotations1, rotations2, rotations3, rotations4;