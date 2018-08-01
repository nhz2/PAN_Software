//
// ADCSCode/ADCSCode.ino
//
// Contributors:
//   Kyle Krol          kpk63@cornell.edu
//   Nathan Zimmerberg  nhz2@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include <i2c_t3_pan.h>
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
/*! Predeclartion of charactar code testing function */
void test_on_char(unsigned char code);

/*! Empties the incoming serial buffer */
void empty_serial() {
  while(Serial.available())
    Serial.read();
}
#endif

/*! Code included for the sun sensor assembly, magnetic torque rods, and
 *  reaction wheels. Note these files need to be included after the definitions
 *  of the TESTING and VERBOSE macros in order for them to have the intended
 *  affect.
 */
#include "mtr.hpp"
#include "ssa.hpp"

void setup() {

  // RWA Potentiometer Analog Read pinMode calls
  // TODO : Move this into an rwa.hpp rwa.cpp pair
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);

  // Initialize all I2C busses
  Wire.begin(I2C_MASTER,0,I2C_PINS_18_19,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);
  Wire1.begin(I2C_MASTER,0,I2C_PINS_37_38,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);
  Wire2.begin(I2C_MASTER,0,I2C_PINS_3_4,I2C_PULLUP_EXT,100000,I2C_OP_MODE_ISR);

  mtr::init();
  ssa::init();

#ifdef VERBOSE
  // Start serial port for verbose output
  Serial.begin(9600);
  // Output all initialization error alert messages
  ssa::verbose_error();
  mtr::verbose_error();
#ifdef TESTING
  // Charactar testing code loop
  while(1) {
    while(Serial.available() < 1);
    unsigned char code = Serial.read();
    empty_serial();
    test_on_char(code);
  }
#endif
#endif
}

void loop() {

  // TODO : fill this buffer

}

/*! ----------------------------------------------------------------------------
 *  All code below this line is included for testing purposes only. It is not
 *  relavent to the flight version of the software.
 */

/*! Test case 's' runs sun sensor ADC tests on their own. The tests records
 *  sun sensor data every read_delay milliseconds and writes it over serial
 *  in the following form until a new charactar is sent over serial:
 *    #time,ssa_data0,...,ssa_data19,ssa_err0,...ssa_err4
 *  To stop the test send any byte over serial.
 */
void test_ssa(unsigned long read_delay) {
  // Data printing function within the scope of this function
  static auto const data_line = []() {
    Serial.print("#" + String(millis()) + ",");
    ssa::verbose_output();
    Serial.println();
  };
  // Print output and read until serial input recieved
  do {
    float v[3];
    data_line();
    delay(read_delay);
    ssa::read(v);
  } while(!Serial.available());
  // Ensure serial is flushed and return
  empty_serial();
}

void test_gyro() {
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
  empty_serial();
}

void test_pot() {
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

/*! Performs a test depending on which charactar code is fed to the function */
void test_on_char(unsigned char code) {
  switch (code) {
    // Performs a sen sensor ADC only test
    case 's':
      Serial.println("@s");
      test_ssa(500);
      break;
    // Performs a gyroscope only test
    case 'g':
      Serial.println("@g");
      // TODO : test function
      break;
    // Potentiometer only test
    case 'p':
      Serial.println("@p");
      // TODO : Actual testing function
      break;
    // Unrecognized charactar code
    default:
      Serial.println("!Unrecognized test charactar code");
  }
}
