#include <Arduino.h>
#include <Global.h>
#include <IMU.h>

#ifndef PRIMARY_H_
#define PRIMARY_H_

    struct states {
        int16_t x;
        int16_t dx;
        int16_t omega;
        float theta;
    };

    // initialize 
    void primary_begin(states*, IMU);

    // read input
    void cart_update(states*);
    void imu_update(states*, IMU*);
    void axis_update(states*);

    // comm with pc
    void serial_receive(states);
    void send(states);  // send sensor data to pc
    void cmd_motor();   // command motor speed
    void restart();
    void calibrate();

#endif