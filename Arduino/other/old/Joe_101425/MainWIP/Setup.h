#pragma once
void startup();
void settozero();

// Expose encoder state to other translation units:
extern volatile long posi1, posi2, posi3, posi4;
extern volatile long rotations1, rotations2, rotations3, rotations4;