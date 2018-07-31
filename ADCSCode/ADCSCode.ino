//
// ADCSCode/ADCSCode.ino
//
// Created by Kyle Krol (kpk63@cornell.edu) on 7/21/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include <i2c_t3_pan.h>
#include <AdafruitADC.hpp>
#include <PololuIMU.hpp>
#include <Potentiometer.hpp>

/*! Leave the macro VERBOSE defined to compile the verbose output functions and
 *  enable verbose output during normal operation.
 */
// #define VERBOSE

/*! Leave the macro TESTING defined to compile into testing mode. If VERBOSE
 *  hasn't previously been defined, defining TESTING will cause VERBOSE to be
 *  defined.
 */
#define TESTING

// Ensure verbose output available for testing mode
#ifdef TESTING
#ifndef VERBOSE
#define VERBOSE
#endif
#endif

/*! Code included for the sun sensor assembly, magnetic torque rods, and
 *  reaction wheels. Note these files need to be included after the definitions
 *  of the TESTING and VERBOSE macros in order for them to have the intended
 *  affect.
 */
#include "mtr.hpp"
#include "ssa.hpp"

#ifdef TESTING
void flush_serial() {
  while(Serial.available())
    Serial.read();
}

void pottest(){
  AD5254 mypot(Wire1,AD5254_ADDR_0);
  delay(100);
  mypot.set_rdac(100,255,100);
  Serial.println(mypot.write_block());
  while(!Serial.available()){
    delay(200);
    Serial.print(String(millis()) + ",");
    Serial.print(String(analogRead(1))+","+
                 String(analogRead(2))+","+
                 String(analogRead(3)));
    Serial.println();

  }
}

void gyrotest(){
  LSM6DS33 mygyro(Wire,DS33_SA0_LOW_ADDRESS,4,4);
  mygyro.i2c_set_timeout(1000);
  delay(50);
  mygyro.power_up();
  delay(100);
  while(!Serial.available()) {
    mygyro.read();
    Serial.print(String(millis()) + ",");
    Serial.print(String(mygyro.g_x())+","+
                 String(mygyro.g_y())+","+
                 String(mygyro.g_z()));
    Serial.println();
    delay(96);
  }
  flush_serial();
}

void suntest(){
  ssa::init();
  Serial.print(String(millis()) + ",");
  ssa::verbose_output();
  Serial.println();
  while(!Serial.available()) {
    float v[3];
    ssa::read(v);
    Serial.print(String(millis()) + ",");
    ssa::verbose_output();
    Serial.println();
    delay(500);
  }
  flush_serial();
}
#endif

void setup() {
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  Wire.begin(I2C_MASTER,0,I2C_PINS_18_19,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);
  Wire1.begin(I2C_MASTER,0,I2C_PINS_37_38,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);
  Wire2.begin(I2C_MASTER,0,I2C_PINS_3_4,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);
#ifdef VERBOSE
  // Start serial port for verbose output
  Serial.begin(9600);
#endif
#ifdef TESTING

  // Testing relies on a user input of a test code (see switch statement)
  while(Serial.available() < 1);
  unsigned char code = Serial.read();
  flush_serial();
  switch (code) {
    case 'a':{ // SSA assembly test alone
      suntest();
      break;
    }
    case 'g':{ // Gyro test alone
      gyrotest();
      break;
    }
    case 'p':{ // Pot test alone
      pottest();
      break;
    }
    default:{
      break;
    }







#endif

  // TODO : Actual full flight code goes here

#ifdef TESTING
    break;
  }
  setup();
#endif
}

void loop() {

}
