#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include "PinDefinitions.h"
#include "Motors.h"
#include "Encoders.h"
#include "Setup.h"
#include "LCD.h"
#include "AccelGyro.h"
#include "PCAServo.h"




//==================================================================================
// --- Prototypes ---
void settozero();        // so startup() can call it
void stopMotors();       // if not already included via Motors.h
void readEncoder1();     // if not already via Encoders.h
void readEncoder2();
void readEncoder3();
void readEncoder4();

// --- Runtime state kept local to this translation unit ---
// Mark ISR-shared items volatile.
volatile long posi1=0, posi2=0, posi3=0, posi4=0;
volatile long rotations1=0, rotations2=0, rotations3=0, rotations4=0;

int  motorSpeedFL=0, motorSpeedFR=0, motorSpeedBL=0, motorSpeedBR=0;

volatile long gyroX=0, gyroY=0, gyroZ=0;
float anglePitch=0.0f, angleRoll=0.0f, angleYaw=0.0f;

float dhtInsideTemp=0.0f, dhtInsideHumidity=0.0f;
float dhtOutsideTemp=0.0f, dhtOutsideHumidity=0.0f;

constexpr int NUM_SONARS = 4;   // match your actual count
long distance[NUM_SONARS] = {0};

float totalDistance = 0.0f;
unsigned long runTime = 0;

int defaultPos[servoCount] = {0, 0, 0, 0, 0};
int servoAngles[servoCount];

//==================================================================================

DHT dht0(DHT0_Pin, DHT0_Type);
DHT dht1(DHT1_Pin, DHT1_Type);

void startup(){

  //Serial Comms
  //================================================================================
  Serial.begin(9600); //initialize serial 9600 communication
  Serial3.begin(115200); // Initialize serial 250000 communication
  Serial.println("Serial Communication Transmitting!"); //Confirmation message

  //Gyro I2C
  //================================================================================
  Wire.begin(); //initalization of I2C  
  IMU::begin(Wire); //init IMU wire begin
  Wire2.begin();       // LCD on SDA2/SCL2 (pins 9/8 on Giga per your defs)

  servosBegin();

  lcd_begin(Wire2, 0x27, 16, 2);  // change 0x27 if your backpack differs
  lcd_set_backlight(255);


  //Initialization of MPU-6050
  //Accelerometer and Gyroscope Data
  Wire.beginTransmission(MPU_ADDR); //Prepares transaction to set variable
  //Wire.write(MPU6050_PWR0);
  Wire.write(0x6B);  // Find power management register
  Wire.write(0x00);     // Wake the MPU-6050 up and clear register
  Wire.endTransmission();

  Serial.println("MPU-6050 Initialized!"); //Confirmation message

  //Motor Pins
  //===============================================================================
  //Set all speed pins as outputs
  pinMode(ENA_FL, OUTPUT); // PWM
  pinMode(ENB_FR, OUTPUT); // PWM
  pinMode(ENA_BR, OUTPUT); // PWM
  pinMode(ENB_BL, OUTPUT); // PWM

  //Set all control pins as outputs
  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN3_FR, OUTPUT);
  pinMode(IN4_FR, OUTPUT);
  pinMode(IN5_BR, OUTPUT);
  pinMode(IN6_BR, OUTPUT);
  pinMode(IN7_BL, OUTPUT);
  pinMode(IN8_BL, OUTPUT);

  Serial.println("Motor Pins Set");

  /*//Servos(positioning)
  //================================================================================
  pwm.begin();
  pwm.setPWMFreq(60);

    // Initialize servo angles to default positions
  for (int i = 0; i < servoCount; i++)
  {
    servoAngles[i] = defaultPositions[i];
    pwm.setPWM(servoPins[i], 0, angleToPulse(servoAngles[i]));
  }
   */

  //Shut motors
  stopMotors();

  Serial.println("Motors Initialized!"); //Confirmation message

  //DHT Sensor - NEEDS WORK
  //===============================================================================
  dht0.begin();
  dht1.begin();
  delay(1500);


  Serial.println("DHT Initialized!"); //Confirmation message

  //Sonar pins
  //===============================================================================
  
  for (int i = 0; i < NUM_SONARS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }


  Serial.println("Sonar Pins Set"); //Confirmation message

  //Encoder pins
  //==============================================================================
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

  //reset everything
  settozero();
  Serial.println("system is zeroed");

}

void settozero() {
  noInterrupts();  // :white_check_mark: Prevent ISRs from updating shared vars while we reset

  // ----------------- Encoders -----------------
  posi1 = rotations1 = 0;
  posi2 = rotations2 = 0;
  posi3 = rotations3 = 0;
  posi4 = rotations4 = 0;

  // ----------------- Motors -------------------
  stopMotors();           // Stop all motor outputs
  motorSpeedFL = 0;       // if you store speed targets
  motorSpeedFR = 0;
  motorSpeedBL = 0;
  motorSpeedBR = 0;



  // ----------------- IMU / Gyro ---------------
  gyroX = 0; gyroY = 0; gyroZ = 0;      // raw sensor accumulators
  anglePitch = 0; angleRoll = 0; angleYaw = 0;

  // ----------------- DHT Sensors --------------
  dhtInsideTemp = 0;
  dhtInsideHumidity = 0;
  dhtOutsideTemp = 0;
  dhtOutsideHumidity = 0;

  // ----------------- Sonar --------------------
  for (int i = 0; i < NUM_SONARS; i++) {
    distance[i] = 0;
  }

  // ----------------- Other State --------------
  totalDistance = 0;
  runTime = 0;

  interrupts();  // :white_check_mark: Re-enable interrupts

  Serial.println("All peripherals and state variables reset to zero!");
}
