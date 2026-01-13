#pragma once
#include <Arduino.h>
#ifndef PINS_H
#define PINS_H


//Current Pin
//============================================================================
#define CURRENT_PIN A8

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
const int trigPins[4] = {15, 39, 51, 37}; //Trig pins F, B, L, R
const int echoPins[4] = {14, 41, 49, 35}; //Echo pins F, B, L, R

//Sonar Directions - PUT THIS SOMEWHERE ELSE
//const char* directions[4] = {"F", "B", "L", "R"}; 

//ENcoder
//===========================================================================================
//Front Right
// #define ENCA_FR 22 
// #define ENCB_FR 24
#define ENCA_FR 23 
#define ENCB_FR 25

//Front Left
#define ENCA_FL 30
#define ENCB_FL 32

//Back Left
#define ENCA_BL 47
#define ENCB_BL 53

//Back Right
// #define ENCA_BR 43
// #define ENCB_BR 45
#define ENCA_BR A1
#define ENCB_BR A2

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
inline constexpr uint8_t DHT1_Pin = 7;

inline constexpr uint8_t DHT0_Type = 11;
inline constexpr uint8_t DHT1_Type = 11;

//I2C MPU-6050 Declaration
inline constexpr uint8_t MPU_ADDR = 0x68;
inline constexpr uint8_t MPU6050_PWR0 = 0x6B;

//I2C Devices
//======================================================================================
//Servo Driver (PCA9685) - WORK IN PROGRESS
/*#define SERVO_SCL1 D101 
#define SERVO_SDA1 D102*/

const int servoCount = 5; //Number of servos
//const int servoPins[] = {D101, D102};//Servo Pins - still saying these pins are wrong?

//buzzer
#define BUZZER A6

//Gyro and Accelerometer (MPU-6050)
#define GYRO_SCL 21
#define GYRO_SDA 20

//LCD Screen
#define LCD_SCL2 8
#define LCD_SDA2 9

//Misc definitions
//=========================================================================================
//Fan control
#define Fan1 A7
#define Fan2 A5

//Button
#define Button 36

#endif
