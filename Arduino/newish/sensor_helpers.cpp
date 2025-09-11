#include "setup_paul.h"

static bool dhtBlinkActive = false;
static unsigned long dhtBlinkUntil = 0;
static const unsigned long LCD_PAGE_MS = 3000; // how long each page stays on screen
static uint8_t s_lcdPage = LCD_HELLO;
static unsigned long s_lcdNextAt = 0;


// LCD print function
// row = 0 or 1 for a 16x2 display
// col = 0–15
void lcdPrint(uint8_t col, uint8_t row, const String& text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void lcdClearRow(uint8_t row) {
  lcd.setCursor(0, row);
  lcd.print("                "); // 16 spaces
}

void lcdTickerInit() {
  s_lcdPage = LCD_HELLO;
  s_lcdNextAt = 0; // force immediate draw on first update
  lcd.clear();
}

// Render the current page
static void lcdRenderPage(uint8_t page) {
  lcd.clear();
  switch (page) {
    case LCD_HELLO:
      lcdPrint(0, 0, "Hello, I am PAUL");
      lcdPrint(0, 1, "Now 20% Smarter");
      break;

    case LCD_ENV: {
      // uses the latest values updated by printDHTIfDue()
      lcdPrint(0, 0, "Temp: " + String(gAmbientTempC, 1) + " C");
      lcdPrint(0, 1, "Hum : " + String(gAmbientRH, 0) + " %");
      break;
    }

    case LCD_MOTOR: {
      // Read relay output state; ON when pin is HIGH (adjust if your relay is inverted)
      bool motorOn = (digitalRead(MOTOR_PWR_RELAY) == HIGH);
      lcdPrint(0, 0, "Motor Power");
      lcdPrint(0, 1, motorOn ? "Status: ON " : "Status: OFF");
      break;
    }
  }
}

void lcdTickerUpdate(unsigned long now) {
  if ((long)(now - s_lcdNextAt) >= 0) {
    lcdRenderPage(s_lcdPage);
    s_lcdPage = (s_lcdPage + 1) % LCD_PAGE_COUNT;
    s_lcdNextAt = now + LCD_PAGE_MS;
  }
}

// Return distance in cm, or 0 if timeout
long readSonarCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // echo duration (µs)
  unsigned long us = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (us == 0) return 0;

  // distance = (c * time) / 2
  // c in m/s → cm/us: c_cm_us = (c * 100) / 1e6 = c / 10000
  float c_ms = currentSpeedOfSoundMS();
  float dist_cm = (c_ms / 10000.0f) * (float)us * 0.5f;

  // clamp/round
  if (dist_cm < 0) dist_cm = 0;
  return (long)(dist_cm + 0.5f);
}

// Read all four sonars and print
void readAndPrintAllSonars() {
  long distF = readSonarCM(SONAR_F_TRIG, SONAR_F_ECHO);
  long distB = readSonarCM(SONAR_B_TRIG, SONAR_B_ECHO);
  long distL = readSonarCM(SONAR_L_TRIG, SONAR_L_ECHO);
  long distR = readSonarCM(SONAR_R_TRIG, SONAR_R_ECHO);

  Serial.print("Front: "); Serial.print(distF); Serial.print(" cm | ");
  Serial.print("Back: ");  Serial.print(distB); Serial.print(" cm | ");
  Serial.print("Left: ");  Serial.print(distL); Serial.print(" cm | ");
  Serial.print("Right: "); Serial.print(distR); Serial.println(" cm");
}

// Non-blocking: double blink on DHT read (every 10 sec)
void printDHTIfDue(unsigned long now) {
  // --- handle ongoing blink sequence ---
  static uint8_t blinkStep = 0;           // 0=idle, 1..4=blink phases
  static unsigned long blinkNextAt = 0;

  if (blinkStep != 0 && (long)(now - blinkNextAt) >= 0) {
    switch (blinkStep) {
      case 1: digitalWrite(LEDB, HIGH); blinkNextAt = now + DHT_BLINK_MS; blinkStep = 2; break; // off after first ON
      case 2: digitalWrite(LEDB, LOW);  blinkNextAt = now + DHT_BLINK_MS; blinkStep = 3; break; // ON second time
      case 3: digitalWrite(LEDB, HIGH); blinkStep = 0; break; // final OFF
    }
  }

  // --- DHT: read every interval ---
  if ((now - lastDHT) < DHT_INTERVAL_MS) return;
  lastDHT = now;

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // °C

  if (isnan(h) || isnan(t)) {
    Serial.println("DHT read failed");
    return;
  }

  // cache environment for sonar
  gAmbientTempC = t;
  gAmbientRH    = h;

  Serial.print("Temp: "); Serial.print(t);
  Serial.print(" °C | Humidity: "); Serial.print(h);
  Serial.println(" %");

  // --- start blink sequence (2 blinks) ---
  digitalWrite(LEDB, LOW);              // ON (active LOW)
  blinkNextAt = now + DHT_BLINK_MS;     // schedule next transition
  blinkStep   = 1;                      // step machine active
}

float currentSpeedOfSoundMS() {
  const float T = gAmbientTempC;
  const float RH = gAmbientRH;
  // Empirical moist-air formula (good to ~±0.5 m/s typical use)
  float c = 331.3f + 0.606f*T + 0.0124f*RH; // m/s
  // Pressure dependence is negligible for speed of sound in ideal gas mixture at these ranges.
  (void)gAmbientPressurePa; // reserved for future refined models
  return c;
}