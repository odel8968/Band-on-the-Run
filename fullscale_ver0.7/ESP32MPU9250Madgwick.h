#ifndef _ESP32MPU9250
#define _ESP32MPU9250

#include <Wire.h>  
#include <Arduino.h>

uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
void I2Cscan();
void MPU9250SelfTest(float * destination);
void accelgyrocalMPU9250(float * dest1, float * dest2);
void initMPU9250();
void initAK8963(float * destination);
void magcalMPU9250(float * dest1, float * dest2, float fixedMagBias[], float fixedMagScale[], int fixedCalibration); 
void readMPU9250Data(int16_t * destination);
void readMagData(int16_t * destination);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void getAres();
void getGres();
void getMres();
int initHeadingReading(float fixedMagBias[], float fixedMagScale[], int fixedCalibration);
int updateHeading(float headings[3]);

#endif
