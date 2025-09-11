#include "setup_paul.h"

DHT dht(DHT_PIN, DHTTYPE);
unsigned long lastDHT = 0;

// Ambient cache (initialize to safe defaults)
float gAmbientTempC      = 20.0f;      // room temp default
float gAmbientRH         = 50.0f;      // mid humidity default
float gAmbientPressurePa = 101325.0f;  // standard atmosphere

constexpr uint8_t LCD_ADDR = 0x27;   // your LCD address
LiquidCrystal_PCF8574 lcd(LCD_ADDR);

void hardwareBegin() {
  // Serial ports
  Serial.begin(115200);      // USB debug (to PC)
  Serial1.begin(PI_BAUD);    // UART to Raspberry Pi on pins 0/1

  // I2C buses
  Wire.begin();              // MPU-6050 on SDA(20)/SCL(21)
  Wire1.begin();             // PCA9685 on SDA1/SCL1
  Wire2.begin();             // I2C LCD on SDA2(9)/SCL2(8)

  //LCD
  lcd.begin(16, 2, Wire2);   // Init LCD with 16 cols, 2 rows, on Wire2
  lcd.setBacklight(255);

  lcdPrint(0, 0, "  Initializing");

  // DHT
  dht.begin();
  lcdPrint(0, 1, "-");
  delay(100);

  // Motor drivers
  pinMode(EN_FRONT_LEFT,  OUTPUT);
  pinMode(EN_FRONT_RIGHT, OUTPUT);
  pinMode(EN_BACK_RIGHT,  OUTPUT);
  pinMode(EN_BACK_LEFT,   OUTPUT);

  pinMode(FR_IN4, OUTPUT);
  pinMode(FR_IN3, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_IN1, OUTPUT);
  pinMode(BL_IN4, OUTPUT);
  pinMode(BL_IN3, OUTPUT);
  pinMode(BR_IN2, OUTPUT);
  pinMode(BR_IN1, OUTPUT);
  lcdPrint(1, 1, "-");
  delay(100);

  // Relay
  pinMode(MOTOR_PWR_RELAY, OUTPUT);
  lcdPrint(2, 1, "-");
  delay(100);

  // Encoders
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP);
  pinMode(ENC_BR_B, INPUT_PULLUP);
  pinMode(ENC_BL_A, INPUT_PULLUP);
  pinMode(ENC_BL_B, INPUT_PULLUP);
  lcdPrint(3, 1, "-");
  delay(100);

  // Sonars
  pinMode(SONAR_F_TRIG, OUTPUT);
  pinMode(SONAR_F_ECHO, INPUT);

  pinMode(SONAR_L_TRIG, OUTPUT);
  pinMode(SONAR_L_ECHO, INPUT);

  pinMode(SONAR_B_TRIG, OUTPUT);
  pinMode(SONAR_B_ECHO, INPUT);

  pinMode(SONAR_R_TRIG, OUTPUT);
  pinMode(SONAR_R_ECHO, INPUT);
  lcdPrint(4, 1, "-");
  delay(100);

  // LEDs
  pinMode(LED_ARDUINO_OK, OUTPUT);
  pinMode(LED_PI_COMM,    OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  lcdPrint(5, 1, "-");
  delay(100);

  // DHT
  pinMode(DHT_PIN, INPUT);       // leave floating/idle (library will drive timing)
  lcdPrint(6, 1, "-");
  delay(100);

  // Button (active LOW)
  pinMode(BTN_USER, INPUT_PULLUP);
  lcdPrint(7, 1, "-");
  delay(100);

  // Fan
  pinMode(FAN_BODY, OUTPUT);
  lcdPrint(8, 1, "-");
  delay(100);

  dht.begin();
  lcdPrint(9, 1, "-");
  delay(100);

  lcdPrint(10, 1, "--");
  delay(100);

  lcdPrint(12, 1, "--");
  delay(100);

  lcdPrint(14, 1, "-");
  delay(100);

  lcdPrint(0, 0, " Init. Complete");
  delay(1000);

  lcd.clear();
}

void allOutputsOff() {
  // Disable motor drivers
  digitalWrite(EN_FRONT_LEFT,  LOW);
  digitalWrite(EN_FRONT_RIGHT, LOW);
  digitalWrite(EN_BACK_RIGHT,  LOW);
  digitalWrite(EN_BACK_LEFT,   LOW);

  // Set all H-bridge inputs LOW
  digitalWrite(FR_IN4, LOW);
  digitalWrite(FR_IN3, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FL_IN1, LOW);
  digitalWrite(BL_IN4, LOW);
  digitalWrite(BL_IN3, LOW);
  digitalWrite(BR_IN2, LOW);
  digitalWrite(BR_IN1, LOW);

  // Relay off (no motor power)
  digitalWrite(MOTOR_PWR_RELAY, LOW);

  // Sonar TRIG low (idle)
  digitalWrite(SONAR_F_TRIG, LOW);
  digitalWrite(SONAR_L_TRIG, LOW);
  digitalWrite(SONAR_B_TRIG, LOW);
  digitalWrite(SONAR_R_TRIG, LOW);

  // LEDs off
  digitalWrite(LED_ARDUINO_OK, LOW);
  digitalWrite(LED_PI_COMM,    LOW);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  // Fan off
  digitalWrite(FAN_BODY, LOW);

  lcdPrint(0, 0, "   Turning Off");
  lcdPrint(0, 1, "    All GPIO");
  delay(1500);
  lcd.clear();
  lcdPrint(0, 0, "      DONE");
  delay(1500);
  lcd.clear();

  lcdTickerInit();   // start the rotating pages
}