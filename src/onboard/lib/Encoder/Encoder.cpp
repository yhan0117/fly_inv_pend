#include "Encoder.h"
using namespace constants;

// create globally defined encoder board objects
EBoard Cart(CART_A, CART_B, CART_ADDRESS);
EBoard Axis(AXIS_A, AXIS_B, AXIS_ADDRESS);

/*********************** Encoder class ***********************/
// Constructors => Initialize encoder parameters and IO pins
Encoder::Encoder(int pin_A, int pin_B) {
  this->A = pin_A;
  this->B = pin_B;
  this->rotating = false;
  this->A_state = false;
  this->B_state = false;
  this->pos = 0; 
  this->dt = (B1 << 15); // MST represents idle or not, 2nd MST represents direction
}

Encoder::Encoder() {
  this->A = 2;
  this->B = 3;
  this->rotating = false;
  this->A_state = false;
  this->B_state = false;
  this->pos = 0; 
  this->dt = (B1 << 15);
}

Encoder::~Encoder() {
  detachInterrupt(digitalPinToInterrupt(this->A));
  detachInterrupt(digitalPinToInterrupt(this->B));
}

// Update per loop
void Encoder::update() {
  // re-enable bouncing
  this->rotating = true;
  // if idle for too long (10ms), update flag (MST of speed)
  if(micros() - this->prev_pulse > 10000) dt |= (B1 << 15);
}

// Reset dials and speed for calibration 
void Encoder::reset() {
  this->pos = 0;  // zero the dial
  this->dt |= (B1 << 15); // flag idle bit
}

// Return speed or position
int16_t Encoder::read(bool flag) { return flag ? this->pos : this->dt;}


/*********************** Encoder board class ***********************/
// Constructors
EBoard::EBoard(int A, int B, uint8_t address) : Encoder(A,B) { this->I2C_address = address; }
EBoard::~EBoard() {}

/*********************** Initialize  ***********************/
/*
  Procedure is as follows
  1. Start I2C protocal
  2. Synchronize with primary board by waiting for it to send starting byte.
     Long flash indicating successful registry of starting signal from primary
  3. Set receive and request events and begin communication with primary
*/
uint8_t EBoard::begin() {
  Wire.begin(this->I2C_address);

  // --------------------------------------------------------------------
  // Synchronize with primary Wait for starting signal
  mode = 0; // wait for sync 
  if (this->I2C_address == CART_ADDRESS)
    Wire.onReceive(cartReceiveEvent);    
  else if (this->I2C_address == AXIS_ADDRESS)
    Wire.onReceive(axisReceiveEvent);    

  // wait for start flag to change
  digitalWrite(LED_BUILTIN, HIGH); 
  while (mode != 1) ;

  Serial.println("Connected");
  // flash to indicate connection with main
  digitalWrite(LED_BUILTIN, LOW); delay(1000); digitalWrite(LED_BUILTIN, HIGH); delay(1000); digitalWrite(LED_BUILTIN, LOW); delay(1000);
 
  // setup Arduino pins
  pinMode(this->A, INPUT_PULLUP);
  pinMode(this->B, INPUT_PULLUP);

  // attach interrupt on encoder pin
  if (this->I2C_address == CART_ADDRESS) {
    attachInterrupt(digitalPinToInterrupt(this->A), cartEncoderA, CHANGE);     
    attachInterrupt(digitalPinToInterrupt(this->B), cartEncoderB, CHANGE);
    Wire.onRequest(cartRequestEvent);  // ISR primary requests a value  
  } else if (this->I2C_address == AXIS_ADDRESS) {
    attachInterrupt(digitalPinToInterrupt(this->A), axisEncoderA, CHANGE);     
    attachInterrupt(digitalPinToInterrupt(this->B), axisEncoderB, CHANGE);
    Wire.onRequest(axisRequestEvent);  // ISR primary requests a value  
  }

  this->prev_pulse = micros();
  return 0;
}
 
/*********************** Encoder ISRs ***********************/
// Hard code two separate sets of ISRs for each instance of the encoder board

// ISR on A changing state 
void cartEncoderA() {
  noInterrupts();  // temporarily disable interrupts
  
  // delay for bouncing to end
  if (Cart.rotating) delayMicroseconds(DEBOUNCE_SPAN);

  // verify transition
  if (digitalRead(Cart.A) !=  Cart.A_state) { 

    Cart.A_state = ! Cart.A_state; // update flag
    
    // adjust counter + if A leads B
    if (Cart.A_state && !Cart.B_state) {
      Cart.pos += 1;
      // if was idle
      if (Cart.dt >> 15) {
        Cart.prev_pulse = micros();
        Cart.dt = ~(B10 << 14);  
      } else {
        Cart.dt = micros() - Cart.prev_pulse;
        Cart.dt |= (B01 << 14); // set direction bit
        Cart.prev_pulse = micros();
      }      
    }
    Cart.rotating = false;  // no more debouncing until loop hits again
  }
  interrupts(); //restart interrupts
}

// ISR on B changing state
void cartEncoderB() {
  noInterrupts();
  if (Cart.rotating) delayMicroseconds(DEBOUNCE_SPAN);
  if (digitalRead(Cart.B) != Cart.B_state) {
    Cart.B_state = !Cart.B_state;
    if (Cart.B_state && !Cart.A_state) {  // adjust counter - 1 if B leads A
      Cart.pos -= 1;
      if (Cart.dt >> 15) {
        Cart.prev_pulse = micros();
        Cart.dt = ~(B11 << 14);  
      } else {
        Cart.dt = micros() - Cart.prev_pulse;
        Cart.dt &= ~(B11 << 14); 
        Cart.prev_pulse = micros();
      }
    }
    Cart.rotating = false;
  }
  interrupts();
}

void axisEncoderA() {
  noInterrupts();  
  if (Axis.rotating) delayMicroseconds(DEBOUNCE_SPAN);
  if (digitalRead(Axis.A) !=  Axis.A_state) { 
    Axis.A_state = ! Axis.A_state;
    if (Axis.A_state && !Axis.B_state) {
      Axis.pos += 1;
      if (Axis.dt >> 15) {
        Axis.prev_pulse = micros();
        Axis.dt = ~(B10 << 14);  
      } else {
        Axis.dt = micros() - Axis.prev_pulse;
        Axis.dt |= (B01 << 14); 
        Axis.prev_pulse = micros();
      }      
    }
    Axis.rotating = false; 
  }
  interrupts(); 
}

void axisEncoderB() {
  noInterrupts();
  if (Axis.rotating) delayMicroseconds(DEBOUNCE_SPAN);
  if (digitalRead(Axis.B) != Axis.B_state) {
    Axis.B_state = !Axis.B_state;
    if (Axis.B_state && !Axis.A_state) {
      Axis.pos -= 1;
      if (Axis.dt >> 15) {
        Axis.prev_pulse = micros();
        Axis.dt = ~(B11 << 14);  
      } else {
        Axis.dt = micros() - Axis.prev_pulse;
        Axis.dt &= ~(B11 << 14);
        Axis.prev_pulse = micros();
      }
    }
    Axis.rotating = false;
  }
  interrupts();
}

/*********************** I2C ISRs ***********************/
/* TBC
  argument -> number of bytes
  valid data should only constain a single byte
*/ 
void cartReceiveEvent(int bytes) {
  if (bytes != 1) {
    // if received more than 1 byte => trash data, clear I2C buffer
    while(Wire.available()) Wire.read();
    return;
  }

  Cart.mode = Wire.read();  // update mode variable
}

// Encode data and send to main through I2C bus
void cartRequestEvent() {
  uint8_t data[2];

  switch ((int)Cart.mode) {
    case 1:   // send start signal to primary and switch to regular mode
      Wire.write(1); // send 1 to primary to indicate start
      Cart.mode = 0; 
      break; 
    case 2:   // send position data
    {
      Wire.write(2); // send 2 to primary to indicate successful read
      uint16_t pos = Cart.read(1);
      data[0] = (uint8_t) pos & 0xFF;
      data[1] = (uint8_t) (pos >> 8) & 0xFF;
      Wire.write(data, 2); // send 2 bytes of pos data -> LST to MST
      Cart.mode = 0;
      break; 
    }
    case 3:
    {   // send speed data
      Wire.write(3); // send 3 to primary to indicate successful read
      uint16_t dt = Cart.read(0);
      data[0] = (uint8_t) dt & 0xFF;
      data[1] = (uint8_t) (dt >> 8) & 0xFF;
      Wire.write(data, 2); // send 2 bytes of dt
      Cart.mode = 0;
      break; 
    }
    case 4:   // reset
      Cart.reset();
      Wire.write(4);
      break;
    case 5:
      Wire.write(5);
      void(* resetFunc)(void) = 0;
      resetFunc();
      break;
    default:
      break;  
  }
  return;
}

void axisReceiveEvent(int bytes) {
  if (bytes != 1) {
    while(Wire.available()) Wire.read();
    return;
  }
  Axis.mode = Wire.read();  
}

void axisRequestEvent() {
  uint8_t data[2];

  switch ((int)Axis.mode) {
    case 1:   
      Wire.write(1);
      Axis.mode = 0; 
      break; 
    case 2:
    {
      Wire.write(2);
      uint16_t pos = Axis.read(1);
      data[0] = (uint8_t) pos & 0xFF;
      data[1] = (uint8_t) (pos >> 8) & 0xFF;
      Wire.write(data, 2);
      Axis.mode = 0;
      break; 
    }
    case 3:
    {   
      Wire.write(3);
      uint16_t dt = Axis.read(0);
      data[0] = (uint8_t) dt & 0xFF;
      data[1] = (uint8_t) (dt >> 8) & 0xFF;
      Wire.write(data, 2); 
      Axis.mode = 0;
      break; 
    }
    case 4:      
      Axis.reset();
      Wire.write(4);
      break;
    case 5:
      Wire.write(5);
      void(* resetFunc)(void) = 0;
      resetFunc();
      break;
    default:
      break;  
  }
  return;
}

/*   // simple test for debugging, turn knob when prompted
  if (self_test) {
    long time = millis();
    boolean state = digitalRead(A);
    boolean change = false;
    while (millis()-time < 5000) {
      if (state != digitalRead(A)) {
        change = true;
        break;
      }
      state = digitalRead(A);
    }

    // if no change is detected, return 1 (error with encoder connection)
    if (!change) {
    }
  }
 */