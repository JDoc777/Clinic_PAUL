#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_PCF8574.h>


// === Function prototypes ===
void hardwareBegin();
void allOutputsOff();
long readSonarCM(uint8_t trigPin, uint8_t echoPin);
void readAndPrintAllSonars();
void printDHTIfDue(unsigned long now);
void lcdPrint(uint8_t col, uint8_t row, const String& text);


// === LCD Sensor Config ===
extern LiquidCrystal_PCF8574 lcd;


// === LCD ticker ===
enum LcdPage : uint8_t { LCD_HELLO = 0, LCD_ENV = 1, LCD_MOTOR = 2, LCD_PAGE_COUNT = 3 };

void lcdTickerInit();
void lcdTickerUpdate(unsigned long now);
void lcdClearRow(uint8_t row);


// === DHT Sensor Config ===
#ifndef DHTTYPE
#define DHTTYPE DHT11
#endif

extern DHT dht;                                 // global DHT object (defined in setup_paul.cpp)
extern unsigned long lastDHT;                   // last time sensor was read
constexpr unsigned long DHT_INTERVAL_MS = 10000UL; // 10 s
constexpr unsigned long DHT_BLINK_MS    = 100UL;     // blue LED blink length (ms)


// ==== Ambient environment cache (used by sonar conversion) ====
extern float gAmbientTempC;          // last °C from DHT
extern float gAmbientRH;             // last %RH from DHT
extern float gAmbientPressurePa;     // Pa (set from another sensor or default 101325)

// Set/override ambient pressure if you add a pressure sensor later.
inline void setAmbientPressurePa(float pa) { gAmbientPressurePa = pa; }

// Compute speed of sound (m/s) from T (°C), RH (%), Pressure (Pa).
float currentSpeedOfSoundMS();


// === Pin definitions ===
// UART to Raspberry Pi (hardware pins RX0=0, TX0=1)
constexpr uint32_t PI_BAUD = 115200;  // adjust if needed

// H-bridge EN pins (PWM-capable, keep LOW = disabled)
constexpr uint8_t EN_FRONT_LEFT   = 5;  // Front dual: ENA (Front Left Wheel)
constexpr uint8_t EN_FRONT_RIGHT  = 4;  // Front dual: ENB (Front Right Wheel)
constexpr uint8_t EN_BACK_RIGHT   = 3;  // Back  dual: ENA (Back Right Wheel)
constexpr uint8_t EN_BACK_LEFT    = 2;  // Back  dual: ENB (Back Left Wheel)

// H-bridge IN pins (LOW = coast/brake off depending on driver)
constexpr uint8_t FR_IN4 = 38;  // Front Right wheel
constexpr uint8_t FR_IN3 = 40;
constexpr uint8_t FL_IN2 = 42;  // Front Left wheel
constexpr uint8_t FL_IN1 = 44;
constexpr uint8_t BL_IN4 = 46;  // Back  Left wheel
constexpr uint8_t BL_IN3 = 48;
constexpr uint8_t BR_IN2 = 50;  // Back  Right wheel
constexpr uint8_t BR_IN1 = 52;

// Motor power relay (LOW = de-energized/off)
constexpr uint8_t MOTOR_PWR_RELAY = 28;

// Encoders (use INPUT_PULLUP by default; change if you have external pull-ups)
constexpr uint8_t ENC_FR_A = 22;
constexpr uint8_t ENC_FR_B = 24;
constexpr uint8_t ENC_FL_A = 30;
constexpr uint8_t ENC_FL_B = 32;
constexpr uint8_t ENC_BR_A = 43;
constexpr uint8_t ENC_BR_B = 45;
constexpr uint8_t ENC_BL_A = 47;
constexpr uint8_t ENC_BL_B = 53;

// Sonars (TRIG = OUTPUT LOW, ECHO = INPUT)
constexpr uint8_t SONAR_F_TRIG = 23;
constexpr uint8_t SONAR_F_ECHO = 25;

constexpr uint8_t SONAR_L_TRIG = 51;
constexpr uint8_t SONAR_L_ECHO = 49;

constexpr uint8_t SONAR_B_TRIG = 39;
constexpr uint8_t SONAR_B_ECHO = 41;

constexpr uint8_t SONAR_R_TRIG = 37;
constexpr uint8_t SONAR_R_ECHO = 35;

// Status LEDs (active HIGH)
constexpr uint8_t LED_ARDUINO_OK = 33;  // "all good"
constexpr uint8_t LED_PI_COMM    = 31;  // Pi communication heartbeat

// DHT sensor (single-wire data; leave as INPUT idle)
constexpr uint8_t DHT_PIN = 34;

// User button: no resistor, other side to GND → use INPUT_PULLUP (active LOW)
constexpr uint8_t BTN_USER = 36;

// Fan control (A7 as digital; LOW = off)
constexpr uint8_t FAN_BODY = A7;