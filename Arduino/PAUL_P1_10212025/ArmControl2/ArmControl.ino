static int lastClawSent = 180;

#include "PCAServo.h"
#include "potentiometer.h"

String input = "";

void setup() {
  Serial.begin(115200);
  delay(3000);


  servosBegin();
  Serial.println("SETUP START");
  
  servosGoHome();
  pot_init();

  Serial.println("READY");
}

void loop() {
  // --- Potentiometer read ---
  PotState s = pot_read();
  pot_print(s);

  // --- Serial command handling ---
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      int base, shoulder, elbow, wrist, claw;

      int count = sscanf(input.c_str(), "%d,%d,%d,%d,%d",
                         &base, &shoulder, &elbow, &wrist, &claw);

      if (count == 5) {
        base     = constrain(base, 0, 180);
        shoulder = constrain(shoulder, 0, 180);
        elbow    = constrain(elbow, 0, 180);
        wrist    = constrain(wrist, 0, 180);
        claw     = constrain(claw, 0, 180);

        // if opening, clear stall latch
        if (claw < lastClawSent - 1) {
          pot_reset_stall();
        }

        int flagNow = pot_get_flag();

        // if stalled, block any further closing
        if (flagNow == 1 && claw > lastClawSent) {
          Serial.print("BLOCKING CLAW: requested ");
          Serial.print(claw);
          Serial.print(" holding ");
          Serial.println(lastClawSent);
          claw = lastClawSent;
        }

        // store final claw command and send to servos
        pot_set_claw_cmd(claw);
        setArms(base, shoulder, elbow, wrist, claw);
        lastClawSent = claw;

        //Serial.print("OK ");
        //Serial.print(base);     Serial.print(",");
        //Serial.print(shoulder); Serial.print(",");
        //Serial.print(elbow);    Serial.print(",");
        //Serial.print(wrist);    Serial.print(",");
        //Serial.println(claw);
      } else {
        Serial.println("BAD INPUT");
      }

      input = "";
    }
    else if (c != '\r') {
      input += c;
    }
  }

  delay(70);
}