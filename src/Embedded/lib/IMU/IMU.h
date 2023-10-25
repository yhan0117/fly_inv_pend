#include <Arduino.h>
#include <Kalman.h>
#include <Global.h>

#ifndef IMU_H_
#define IMU_H_

    // inherit from the Kalman filter class
    class IMU : Kalman{
    private:
        // accelerometer variables
        float accY, accZ;

        // gyroscope variables
        float gyroX;
        long gyroTimer;

    public:
        IMU();
        ~IMU();
        uint8_t begin();
        float read();
    };
    
#endif