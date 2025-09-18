#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include "PinDefinitions.h"

DHT dht0(DHT0_Pin, DHT0_Type);
DHT dht1(DHT1_Pin, DHT1_Type);


void startup(){

  //Serial Comms
  //================================================================================
  Serial.begin(9600); //initialize serial 9600 communication
  Serial3.begin(250000); // Initialize serial 250000 communication
  Serial.println("Serial Communication Transmitting!"); //Confirmation message

  //Gyro I2C
  //================================================================================
  Wire.begin(); //initalization of I2C

  //Initialization of MPU-6050
  Wire.beginTransmission(MPU_ADDR); //Prepares transaction to set variable
  Wire.write(0x6B);  // Find power management register
  Wire.write(0);     // Wake the MPU-6050 up and clear register
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

  //Shut motors
  stopMotors();

  Serial.println("Motors Initialized!"); //Confirmation message

  //DHT Sensor - NEEDS WORK
  //===============================================================================
  dht0.begin();
  dht1.begin();

  Serial.println("DHT Initialized!"); //Confirmation message

  //Sonar pins
  //===============================================================================
  
  for (int i = 0; i < 4; i++) { //Sonar loop
    pinMode(trigPins[i], OUTPUT); //Trig pins Output
    pinMode(echoPins[i], INPUT); //Echo pins Input
  }

  Serial.println("Sonar Pins Set"); //Confirmation message

  //Encoder pins
  //==============================================================================
  pinMode(ENCA_FR, INPUT);
  pinMode(ENCB_FR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_FR), readEncoder1, RISING);

  pinMode(ENCA_FL, INPUT);
  pinMode(ENCB_FL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_FL), readEncoder2, RISING);

  pinMode(ENCA_BL, INPUT);
  pinMode(ENCB_BL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_BL), readEncoder3, RISING);

  pinMode(ENCA_BR, INPUT);
  pinMode(ENCB_BR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_BR), readEncoder4, RISING);

  Serial.println("Encoders Initialized");

}

void settozero(){

}