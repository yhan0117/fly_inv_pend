/* Header file for all global variables */
#include <Arduino.h>
#include <Wire.h>

#ifndef GLOBAL_H_
#define GLOBAL_H_
    namespace constants {
        const uint8_t CART_ADDRESS = 0x08;  // I2C address of Cart
        const uint8_t AXIS_ADDRESS = 0x09;  // I2C address of X axis board
        const uint8_t IMU_ADDRESS = 0x68;   // I2C address of IMU
        const int SENSOR_TIMEOUT = 2000;    // timeout to wait for sensor input
        const int DEBOUNCE_SPAN = 800;      // wait time for debounce to end, microseconds
        
        // encoder pins
        const int CART_A = 2;      // Encoder pin A for cart
        const int CART_B = 3;      // Encoder pin B for cart
        const int AXIS_A = 2;      // Encoder pin A for x axis board
        const int AXIS_B = 3;      // Encoder pin B for x axis board

        // motor & driver pins
        const int RPWM = 5; // Right power control, connect to IBT-2 pin 1 (RPWM)
        const int LPWM = 6; // Left power control, connect to IBT-2 pin 2 (LPWM)
        const int LEN = 8; // IBT-2 left enable, set high to arm
        const int REN = 9; // IBT-2 right enable, set high to arm
    }; // constants

    // Send once through bus 
    uint8_t i2cWrite(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
    uint8_t i2cRead(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

    // Send until timeout
    uint8_t wait_until_timeout(uint8_t targetAddress, uint8_t registerAddress, uint8_t *data, uint8_t nbytes, char op);
    uint8_t wait_until_timeout(uint8_t targetAddress, uint8_t registerAddress, uint8_t data, char op);
#endif