
#define ENCA_FR 22 
#define ENCB_FR 24

//Front Left
#define ENCA_FL 30
#define ENCB_FL 32

//Back Left
#define ENCA_BL 47
#define ENCB_BL 53

//Back Right
// #define ENCA_BR 43
// #define ENCB_BR 45
#define ENCA_BR A0
#define ENCB_BR A2

long posi1, posi2, posi3, posi4;
long rotations1, rotations2, rotations3, rotations4;
long PULSES_PER_REV = 111;

void setup() {

  Serial.begin(9600); //initialize serial 9600 communication

  pinMode(ENCA_FR, INPUT_PULLUP);
  pinMode(ENCB_FR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoder1, FALLING);

  pinMode(ENCA_FL, INPUT_PULLUP);
  pinMode(ENCB_FL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoder2, RISING);

  pinMode(ENCA_BL, INPUT_PULLUP);
  pinMode(ENCB_BL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_BL), readEncoder3, RISING);

  pinMode(ENCA_BR, INPUT_PULLUP);
  pinMode(ENCB_BR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_BR), readEncoder4, FALLING);

  Serial.println("Encoders Initialized");

}

void loop() {

  // Print Encoder
  Serial.print("FR: "); Serial.println(posi1);
  Serial.print("FL: ");  Serial.println(posi2);
  Serial.print("RL: "); Serial.println(posi3);
  Serial.print("RR: ");  Serial.println(posi4);

  delay(100);

}

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