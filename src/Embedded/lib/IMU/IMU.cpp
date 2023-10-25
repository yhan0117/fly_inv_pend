#include "IMU.h"
using namespace constants;

uint8_t rcode; // returned exception code
uint8_t i2cData[14];  // buffer for I2C data


/*********************** Constructors ***********************/
IMU::IMU() {
  // Settings
  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2
}

IMU::~IMU() {
// reset?
}


/*********************** Initialize IMU and I2C comm***********************/
uint8_t IMU::begin() {
  // Write settings to all four registers at once
  if ((rcode = wait_until_timeout(IMU_ADDRESS, 0x19, i2cData, 4, 'w'))) return rcode;
  
  // PLL with X axis gyroscope reference and disable sleep mode
  if ((rcode = wait_until_timeout(IMU_ADDRESS, 0x6B, (uint8_t)0x01, 'w'))) return rcode;

  // Read "WHO_AM_I" register
  if ((rcode = wait_until_timeout(IMU_ADDRESS, 0x75, i2cData, 1, 'r'))) return rcode;
  if (i2cData[0] != 0x68) return 7; // fail to id sensor 

  delay(100); // Wait for sensor to stabilize

  // obtain starting angle
  if ((rcode = wait_until_timeout(IMU_ADDRESS, 0x3B, i2cData, 6, 'r'))) return rcode;

  // estimate current angle and set as starting angle
  this->accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  this->accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]); 

  // set starting angle and timer for gyro
  this->angle = atan2(accY, accZ) * RAD_TO_DEG;
  this->gyroTimer = micros();

  return 0;
}


/*********************** Read IMU output registers and filter data ***********************/
/* 
  1. Fetch data from IMU
  2. Perform necessary unit conversions and calculations
  3. Apply Kalman filter
  4. Update angle data
  5. Return angle
*/
float IMU::read() {
  // read acel & gyro values, dont update if read fails
  if ((rcode = wait_until_timeout(IMU_ADDRESS, 0x3B, i2cData, 6, 'r'))) return -1;

  // decode read data
  this->accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  this->accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  this->gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);

  // calculate delta time for integration
  double dt = (double)(micros() - this->gyroTimer) / 1000000; 
  this->gyroTimer = micros();

  // unit conversion
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double gyroXrate = gyroX / 131.0; // Convert to deg/s

  // fix transition problem between -180 and 180 degrees
  if ((roll < -90 && this->angle > 90) || (roll > 90 && this->angle < -90))
    this->angle = roll;
  // calculate and update the angle using a Kalman filter
  else     
    this->angle = this->getAngle(roll, gyroXrate, dt); 

  delay(2);
  return this->angle;
}

