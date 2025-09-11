#include PinDefinitions.h
#include Setup.h

void Setup(){

  //Serial Comms
  //================================================================================
  Serial.begin(9600); //initialize serial 9600 communication
  Serial3.begin(250000); // Initialize serial 250000 communication
  Serial.println("Serial Communication Transmitting!") //Confirmation message

  //Gyro I2C
  //================================================================================
  Wire.begin(); //initalization of I2C

  //Initialization of MPU-6050
  Wire.beginTransmission(MPU_ADDR); //Prepares transaction to set variable
  Wire.write(0x6B);  // Find power management register
  Wire.write(0);     // Wake the MPU-6050 up and clear register
  Wire.endTransmission();

  Serial.println("MPU-6050 Initialized!") //Confirmation message
}