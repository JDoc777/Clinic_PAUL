#pragma once
#include <Arduino.h>
#ifndef PINS_H
#define PINS_H



//Motors/Wheels
//==========================================================================================
//Wheels
#define ENA_FL 5 //Front left wheel - pin 5
#define ENB_FR 4 //Front right wheel - pin 4
#define ENA_BR 3 //Back right wheel - pin 3
#define ENB_BL 2 //Back left wheel - pin 2

//Relay motor
#define Relay_motors 28

//Dual H-Bridge FRONT
#define IN1_FL 44 //Front left
#define IN2_FL 42 //Front left
#define IN3_FR 40 //Front back
#define IN4_FR 38 //Front back

//Dual H-Bridge BACK
#define IN5_BR 52 //Back right
#define IN6_BR 50 //Back right
#define IN7_BL 48 //Back left
#define IN8_BL 46 //Back left

//Sonar
//============================================================================================
//Sonar (Ultrasonic) Trig and Echo - // F = Front, B = Back, L = Left, R = Right
const int trigPins[4] = {23, 39, 51, 37}; //Trig pins F, B, L, R
const int echoPins[4] = {25, 41, 49, 35}; //Echo pins F, B, L, R

//Sonar Directions - PUT THIS SOMEWHERE ELSE
//const char* directions[4] = {"F", "B", "L", "R"}; 

//ENcoder
//===========================================================================================
//Front Right
#define ENCA_FR 22 
#define ENCB_FR 24

//Front Left
#define ENCA_FL 30
#define ENCB_FL 32

//Back Left
#define ENCA_BL 47
#define ENCB_BL 53

//Back Right
#define ENCA_BR 43
#define ENCB_BR 45

//UART
//========================================================================================
//Pi Connections
#define TX0 1 //To Pi
#define RX0 0 //From Pi

//LED status
#define LED_Arduino 33
#define LED_Pi 31

//DHT - NEEDS WORK
inline constexpr uint8_t DHT0_Pin = 34;

inline constexpr uint8_t DHT0_Type = 11;

//I2C MPU-6050 Declaration
inline constexpr uint8_t MPU_ADDR = 0x68;
inline constexpr uint8_t MPU6050_PWR0 = 0x6B;

//I2C Devices
//======================================================================================
//Servo Driver (PCA9685)
const int servoCount = 5; //Number of servos
//const int servoPins[] = {SDA1, SDL1};//Servo Pins - still saying these pins are wrong?

//Gyro and Accelerometer (MPU-6050)
#define GYRO_SCL 21
#define GYRO_SDA 20

//LCD Screen
#define LCD_SCL2 8
#define LCD_SDA2 9

//Misc definitions
//=========================================================================================
//Fan control
#define Fan A7

//Button
#define Button 36

#endif
