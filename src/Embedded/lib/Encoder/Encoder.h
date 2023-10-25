#include <Arduino.h>
#include <Global.h>
using namespace constants;


#ifndef ENCODER_H_
#define ENCODER_H_

    // encoder class
    class Encoder {
    public:
        // Encoder Variables 
        int A,B;    // pins
        volatile boolean rotating;  // debounce management
        volatile boolean A_state, B_state;  // state flag

        volatile long prev_pulse;   // time of previous pulse
        volatile int16_t pos; // counter for dial
        volatile int16_t dt;  // time between successive pulses
        
        Encoder(int A, int B);
        Encoder();
        ~Encoder();

        void update();   // update debounce management and check idle
        int16_t read(bool);  // returns speed(0) or position (1)
        void reset();    // reset position and speed
    };

    class EBoard : public Encoder {
    private:
        uint8_t I2C_address;

    public:
        volatile uint8_t mode; // mode for synchronization and to determine what to send upon a request
        EBoard(int, int, uint8_t);
        ~EBoard();
        uint8_t begin();
    };

    // interrupt routine prototypes
    void cartEncoderA();
    void cartEncoderB();
    void axisEncoderA();
    void axisEncoderB();
    void cartReceiveEvent(int);
    void cartRequestEvent();
    void axisReceiveEvent(int);
    void axisRequestEvent();
#endif