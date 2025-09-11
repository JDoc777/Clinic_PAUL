/*  PAUL.P1 Robot — Arduino Giga R1
 *  Contributors - Justin Dougherty
 *  Date - 8/28/25
 *. Time - 03:23
 */

#include "setup_paul.h"
#include "wheels.h"

void setup() {
  hardwareBegin();   // sets up all pins and buses
  allOutputsOff();   // ensure everything starts off
  encodersBegin();
}

void loop() {
  unsigned long now = millis();
  //readAndPrintAllSonars();          // fast loop
  printDHTIfDue(now);          // only prints every 10s
  lcdTickerUpdate(now);    // rotates the LCD pages every 2.5s

  // Print counts every 500 ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 50) {
    encodersPrintCounts();
    lastPrint = now;
  }
}