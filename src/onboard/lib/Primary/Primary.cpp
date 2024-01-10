#include "Primary.h"
using namespace constants;


/*********************** Initialize Primary board ***********************/
/*
  1. Wait for prompt to start from user through Serial port 
  2. Prompt cart board to begin by sending 1 and confirm that it sends back 1
  3. Prompt x axis board to start
  4. Setup Kalman instance and start IMU
  5. Setup motor and state variables 
*/
void primary_begin(states* curState, IMU myIMU) {  
  Serial.begin(115200); // initialize serial port as communication interface with PC
  Serial.setTimeout(20);  
  Wire.begin(); // join I2C bus as master

  // variables and communication buffer
  uint8_t data[10];
  uint8_t rcode;

  // --------------------------------------------------------------------
  // wait for user prompt to start (through Serial port)
  digitalWrite(LED_BUILTIN, HIGH);  
  while (Serial.read() != '1') ; 
  Serial.write('1');
  digitalWrite(LED_BUILTIN, LOW);  
  while (Serial.available()) Serial.read();  // clear serial buffer

  // --------------------------------------------------------------------
  // prompt cart to start by sending 1 (in the register address field)
  // while ((rcode = i2cRead(CART_ADDRESS, 1, data, 1)) || data[0] != 1) {
  //   delay(1000);
  //   Serial.write(rcode | (0b1 << 7)); // flag MST high to show its an cart error
  // } 
  // Serial.write('1');

  // --------------------------------------------------------------------
  // prompt x axis board to start
  while ((rcode = i2cRead(AXIS_ADDRESS, 1, data, 1)) || data[0] != 1) {
    delay(1000);
    Serial.write(rcode | (0b1 << 6));
  } 
  Serial.write('1');

  // --------------------------------------------------------------------
  // create IMU instance and start
  // while ((rcode = myIMU.begin()))  Serial.write(rcode | (0b1 << 5));  // keep trying until success
  // Serial.write('1');

  // --------------------------------------------------------------------
  // Setup motor variables
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  
  // initialize state variables
  curState->x = 0;
  curState->dx = 0;
  curState->omega = 0;
  curState->theta = 0;

  Serial.write('2'); // send back to user
}


/*********************** update data from cart ***********************/
void cart_update(states* curState) {
  uint8_t data[3];
  uint8_t rcode;
  // request pole angular speed from cart board
  while ((rcode = i2cRead(CART_ADDRESS, 3, data, 3)) || data[0] != 3) {
    delay(1000);
    Serial.write(rcode & (0b1 << 7));
  } 
  curState->omega = (int16_t)((data[2] << 8 | data[1]) & 0xffff);
}


/*********************** update data from x axis ***********************/
void axis_update(states* curState) {
  uint8_t data[3];
  uint8_t rcode;
  // request cart x position from x axis board
  while ((rcode = i2cRead(AXIS_ADDRESS, 2, data, 3)) || data[0] != 2) {
    delay(1000);
    Serial.write(rcode & (0b1 << 6));
  } 
  curState->x = (int16_t)((data[2] << 8 | data[1]) & 0xffff);

  // request cart x velocity from x axis board
  while ((rcode = i2cRead(AXIS_ADDRESS, 3, data, 3)) || data[0] != 3) {
    delay(1000);
    Serial.write(rcode & (0b1 << 6));
  } 
  curState->dx = (int16_t)((data[2] << 8 | data[1]) & 0xffff);
}


/*********************** update IMU data ***********************/
void imu_update(states* curState, IMU* myIMU) {
  float theta = myIMU->read();
  if (theta != -1)
    curState->theta = theta;
  else 
    Serial.write('9' & (0b1 << 5));  // error reading IMU data
}


/*********************** send data to PC upon request ***********************/
void send(states curState) {
  Serial.write('3');
  Serial.write((uint8_t*) &curState, sizeof(states));
}

/*********************** send data to PC upon request ***********************/
void calibrate() {
  uint8_t data;
  i2cRead(CART_ADDRESS, 4, &data, 1);
  if (data != 4) Serial.write('9' & (0b1 << 5));  // could not calibrate
  else Serial.write('4');
}

/*********************** send data to PC upon request ***********************/
void restart() {
  // turn off motor
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  uint8_t data;
  // prompt x
  i2cRead(AXIS_ADDRESS, 5, &data, 1);
  if (data != 5) Serial.write('9' & (0b1 << 6));  // could not calibrate

  // prompt cart
  i2cRead(CART_ADDRESS, 5, &data, 1);
  if (data != 5) Serial.write('9' & (0b1 << 7));  // could not restart

  // send back pc confirmation
  Serial.println('5');
  void(* resetFunc)(void) = 0;
  resetFunc();
}

/*********************** update motor pwr/ pwm ***********************/
void cmd_motor() {
  int8_t pwr = Serial.read(); // get desired input

  if (pwr >= 0) {
    // CCW (L)
    analogWrite(RPWM, 0);
    analogWrite(LPWM, abs((int)pwr));
  } else {
    // CW (R)
    analogWrite(LPWM, 0);
    analogWrite(RPWM, abs((int)pwr));
  }
}

/*********************** decode command from pc ***********************/
void serial_receive(states curState) {
  switch (Serial.read()) {
  case '2':
    // data request
    send(curState);
    break;
  case '3':
    // zero encoder pos counter
    calibrate();
    break;
  case '4':
    // restart all boards
    restart();
    break;
  case '5':
    // set motor pwr
    cmd_motor();    
    break;
  default:
    break;
  }

  while (Serial.available()) Serial.read();  // clear serial buffer
}


