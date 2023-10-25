#include "Global.h"

const uint16_t TIMEOUT = 1000; // Used to check for errors in I2C communication
 
uint8_t i2cWrite(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(targetAddress);
  Wire.write(registerAddress);
  if (length > 0) {
    Wire.write(data, length);
  }
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  return rcode; 
}

uint8_t i2cRead(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(targetAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus ?

  // fail to read (initiate I2C bus with peripheral)
  if (rcode) {
    return 8; 
  }

  Wire.requestFrom(targetAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    // read
    if (Wire.available()){
      data[i] = Wire.read();
    }
    else {
      timeOutTimer = micros();
      // enter wait until timeout
      while (((micros() - timeOutTimer) < TIMEOUT) && !Wire.available());

      if (Wire.available())
        data[i] = Wire.read();
      else {  // timeout
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }

  // clear buffer
  while(Wire.available()){
    Wire.read();
  }

  return 0; // Success
}

/*********************** Make wait a function ***********************/
uint8_t wait_until_timeout(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes, char op) {
  int code; 
  if (op == 'w') 
    code = i2cWrite(targetAddress, registerAddress, data, nbytes, false);
  else if (op == 'r') 
    code = i2cRead(targetAddress, registerAddress, data, nbytes);  
  
  // start timing 
  long time = millis();
  while (millis()-time < TIMEOUT && code) {
    if (op == 'w') 
      code = i2cWrite(targetAddress, registerAddress, data, nbytes, false);
    else if (op == 'r') 
      code = i2cRead(targetAddress, registerAddress, data, nbytes);  
  }

  return code;
}

uint8_t wait_until_timeout(uint8_t targetAddress, uint8_t registerAddress, uint8_t data, char op) {
  int code = i2cWrite(targetAddress, registerAddress, &data, 1, false);
  
  // start timing 
  long time = millis();
  while (millis()-time < TIMEOUT && code) {
    code = i2cWrite(targetAddress, registerAddress, &data, 1, false);
  }

  return code;
}