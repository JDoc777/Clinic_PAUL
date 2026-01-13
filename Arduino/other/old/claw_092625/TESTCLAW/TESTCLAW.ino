#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* ===================== User Config ===================== */
constexpr uint8_t  PCA_ADDR      = 0x40;     // change if A0/A1/A2 are strapped
constexpr float    SERVO_FREQ_HZ = 50.0f;    // try 60.0f if your servos prefer it
constexpr uint32_t PCA_OSC_HZ    = 27000000; // Adafruit boards use 25–27MHz; 27MHz is typical
constexpr uint16_t SERVO_MIN_US  = 650;      // calibrate to your servos
constexpr uint16_t SERVO_MAX_US  = 2350;     // calibrate to your servos
constexpr int      STEP_MIN      = 1;
constexpr int      STEP_MAX      = 20;

/* Channel mapping: set these to the PCA9685 outputs you wired */
constexpr uint8_t CH_BASE      = 15;
constexpr uint8_t CH_SHOULDER  = 14;
constexpr uint8_t CH_ELBOW     = 13;
constexpr uint8_t CH_WRIST_P   = 9;
constexpr uint8_t CH_CLAW      = 11;   // <-- add your claw channel

/* ======= Joint setup (active joints) ======= */
struct JointCfg {
  uint8_t ch;
  int     home;
  int     minDeg;
  int     maxDeg;
  int     deg;
};

enum { BASE, SHOULDER, ELBOW, WRIST_P, CLAW, JOINT_COUNT };

JointCfg J[JOINT_COUNT] = {
  /* BASE     */ { CH_BASE,     90,   0, 180, 0 },
  /* SHOULDER */ { CH_SHOULDER, 90,  10, 170, 0 },
  /* ELBOW    */ { CH_ELBOW,    90,  10, 170, 0 },
  /* WRIST_P  */ { CH_WRIST_P,  90,  20, 160, 0 },
  /* CLAW     */ { CH_CLAW,     30,  10, 120, 0 },  // tune min/max/home for your claw
};

int stepDeg = 5;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

/* ===================== Helpers ===================== */
static inline int clamp(int v, int lo, int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

static inline uint16_t angleToMicros(int deg){
  deg = clamp(deg, 0, 180);
  const uint32_t span = (uint32_t)(SERVO_MAX_US - SERVO_MIN_US);
  return (uint16_t)(SERVO_MIN_US + (uint32_t)deg * span / 180UL);
}

// Robust pulse writer that works regardless of library version.
// Converts microseconds -> 12-bit ticks at the configured frequency.
static inline void writePulseUS(uint8_t ch, uint16_t us){
  // ticks = us * freq(Hz) * 4096 / 1e6
  float ticksF = (float)us * SERVO_FREQ_HZ * 4096.0f / 1000000.0f;
  uint16_t ticks = (uint16_t)clamp((int)(ticksF + 0.5f), 0, 4095);
  pwm.setPWM(ch, 0, ticks);
}

static inline void writeAngle(uint8_t ch, int deg){
  writePulseUS(ch, angleToMicros(deg));
}

static void goHome(){
  for (int i = 0; i < JOINT_COUNT; ++i) {
    J[i].deg = J[i].home;
    writeAngle(J[i].ch, J[i].deg);
  }
}

static void printAll(){
  Serial.print(F("Base="));     Serial.print(J[BASE].deg);
  Serial.print(F("  Shoulder=")); Serial.print(J[SHOULDER].deg);
  Serial.print(F("  Elbow="));    Serial.print(J[ELBOW].deg);
  Serial.print(F("  WristP="));   Serial.print(J[WRIST_P].deg);
  Serial.print(F("  Claw="));     Serial.print(J[CLAW].deg);
  Serial.print(F("  [step="));    Serial.print(stepDeg); Serial.println(']');
}

static void moveBy(int idx, int delta){
  J[idx].deg = clamp(J[idx].deg + delta, J[idx].minDeg, J[idx].maxDeg);
  writeAngle(J[idx].ch, J[idx].deg);
  printAll();
}

static void setDeg(int idx, int deg){
  J[idx].deg = clamp(deg, J[idx].minDeg, J[idx].maxDeg);
  writeAngle(J[idx].ch, J[idx].deg);
  printAll();
}

static void printHelp(){
  Serial.println(F("\n=== PAUL Arm Teleop (PCA9685) — 5 joints ==="));
  Serial.println(F("BASE:      J/L   (left/right)"));
  Serial.println(F("SHOULDER:  I/K   (up/down)"));
  Serial.println(F("ELBOW:     A/D   (+/-)"));
  Serial.println(F("WRIST P:   W/S   (up/down)"));
  Serial.println(F("CLAW:      Q/E   (close/open)"));
  Serial.println(F("GLOBAL:    R=home, [ step--, ] step++, T=self-test, H/?=help"));
}

/* ========== Diagnostics ========== */
static void i2cScan(){
  Serial.println(F("I2C scan..."));
  uint8_t count=0;
  for(uint8_t addr=1; addr<127; ++addr){
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if(err==0){
      Serial.print(F("  Found device at 0x"));
      if(addr<16) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    }
  }
  if(!count) Serial.println(F("  No I2C devices found."));
}

static void selfTest(){
  Serial.println(F("Self-test: wiggle each joint 3×"));
  for(int i=0;i<JOINT_COUNT;++i){
    int h = J[i].home;
    int lo = clamp(h-20, J[i].minDeg, J[i].maxDeg);
    int hi = clamp(h+20, J[i].minDeg, J[i].maxDeg);
    // go to home first
    writeAngle(J[i].ch, h); delay(250);
    for(int k=0;k<3;++k){
      writeAngle(J[i].ch, lo); delay(250);
      writeAngle(J[i].ch, hi); delay(250);
    }
    writeAngle(J[i].ch, h); delay(250);
  }
  Serial.println(F("Self-test done."));
}

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Make sure the oscillator and freq are set (prevents wrong pulse widths)
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  i2cScan();   // you should see 0x40 here; if not, wiring/address is wrong

  goHome();
  printHelp();
  printAll();
}

void loop() {
  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') return;  // ignore CR/LF
    if (c >= 'a' && c <= 'z') c -= 32;   // normalize to uppercase

    switch (c) {
      // Base
      case 'J': moveBy(BASE, -stepDeg); break;
      case 'L': moveBy(BASE, +stepDeg); break;

      // Shoulder
      case 'I': moveBy(SHOULDER, +stepDeg); break;
      case 'K': moveBy(SHOULDER, -stepDeg); break;

      // Elbow
      case 'A': moveBy(ELBOW, +stepDeg); break;
      case 'D': moveBy(ELBOW, -stepDeg); break;

      // Wrist pitch
      case 'W': moveBy(WRIST_P, +stepDeg); break;
      case 'S': moveBy(WRIST_P, -stepDeg); break;

      // Claw
      case 'Q': moveBy(CLAW, -stepDeg); break;  // typically "close"
      case 'E': moveBy(CLAW, +stepDeg); break;  // typically "open"

      // Global / tools
      case 'R': goHome(); printAll(); break;
      case '[': stepDeg = clamp(stepDeg - 1, STEP_MIN, STEP_MAX);
                Serial.print(F("Step=")); Serial.println(stepDeg); break;
      case ']': stepDeg = clamp(stepDeg + 1, STEP_MIN, STEP_MAX);
                Serial.print(F("Step=")); Serial.println(stepDeg); break;
      case 'T': selfTest(); break;
      case 'H':
      case '?': printHelp(); break;

      default: break;
    }
  }
  delay(5);
}


