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
#define VERBOSE

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

  // Initialize all I2C busses
  Wire.begin(I2C_MASTER,0,I2C_PINS_18_19,I2C_PULLUP_EXT,400000,I2C_OP_MODE_ISR);
  Wire1.begin(I2C_MASTER,0,I2C_PINS_37_38,I2C_PULLUP_EXT,400000,I2C_OP_MODE_ISR);
  Wire2.begin(I2C_MASTER,0,I2C_PINS_3_4,I2C_PULLUP_EXT,400000,I2C_OP_MODE_ISR);

  // Initialize all ADCS assemblies
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

  // TODO : Put actual flight code here

}

/*! ----------------------------------------------------------------------------
 *  All code below this line is included for testing purposes only. It is not
 *  relavent to the flight version of the software.
 */
 #ifdef TESTING

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
  AD5254 mypot(Wire1,AD5254_ADDR_1);
  delay(500);
  mypot.set_rdac(0,0,0);
  Serial.println(mypot.write_block());
  while(!Serial.available()){
    delay(500);
    Serial.print(String(millis()) + ",");
    Serial.print(String(analogRead(1))+","+
                 String(analogRead(2))+","+
                 String(analogRead(3)));
    Serial.println();
  }
}

void test_dac() {
  //from https://www.pjrc.com/teensy/teensy31.html
  analogWriteResolution(12);
  analogWrite(A22, 4050);
  analogWrite(A21, 4050);
  while(true){}
}


/*! Test for simulink filters of imu data
    Data written to serial when a serial query of 'n' is made
    from the query block in simulink.

    The data is formated as a comma sepperated list of ints
      #new_G,Gx,Gy,Gz,new_M,Mx,My,Mz,new_XL,XLx,XLy,XLz/n
    where new_* is 0 if no new data is available,
    and 1 if the data is valid .
      */

void test_imu_filter() {
  //start up gyro, accel, and mag.
  LSM6DS33 mygyro(Wire,DS33_SA0_LOW_ADDRESS,4,4);
  //mygyro.i2c_set_timeout(1000);
  LIS3MDL mymag(Wire,LIS3MDL_ADDR_LOW);
  delay(500);
  mymag.set_sample_frequency(LIS3MDL_SF_80);
  mymag.i2c_set_timeout(100);
  mygyro.i2c_set_timeout(100);
  delay(100);
  mymag.set_xy_performance(LIS3MDL_PERF_ULTRA);
  mymag.set_z_performance(LIS3MDL_PERF_ULTRA);
  mygyro.power_up();
  mymag.read();
  delay(100);

  empty_serial();
  while(1){
    //wait until a n is recieved
    while(!('n'==Serial.read()));
    //wait until gyro data is ready
    while(!(mygyro.readReg(mygyro.STATUS_REG)&2)){}
    mygyro.read();
    Serial.print("#");
    Serial.print(String((int)mygyro.get_tda())+",");
    Serial.print(String(mygyro.g_x())+",");
    Serial.print(String(mygyro.g_y())+",");
    Serial.print(String(mygyro.g_z())+",");

    mymag.read();
    Serial.print("1,");
    Serial.print(String(mymag.x())+",");
    Serial.print(String(mymag.y())+",");
    Serial.print(String(mymag.z())+",");
    Serial.print(String((int)mygyro.get_xlda())+",");
    Serial.print(String(mygyro.xl_x())+",");
    Serial.print(String(mygyro.xl_y())+",");
    Serial.print(String(mygyro.xl_z()));
    Serial.println();
  }
}

void test_x() {
  //RWA x setup
  int SpeedSet = 23;
  int CCW = 39;
  int CW = 26;
  float speedA;
  int Aread = 14;
  float voltageA;
  int gate=0;
  int gate1=0;
  int gate2=0;
  int gate3=0;
  int detumble_cmd;
  float max_speed=2000.0;
  int min_speed_cmd=29;
  int max_speed_cmd=230;
  int speed_cmd=30;
  int max_accel=2000;
  int accel=0;
  int accel_cmd=255;
  int ACC=1;
  AD5254 pot(Wire1,AD5254_ADDR_1);
  int delta_accel=100;
  float time_delay;
  int counter1;
  int counter2;
  pinMode(CW,OUTPUT);
  pinMode(CCW,OUTPUT);
  pinMode(SpeedSet,OUTPUT);
  pinMode(ACC,INPUT);
  pinMode(A2,INPUT);

  digitalWrite(CCW,LOW); // Only one of these can be high at a time
  digitalWrite(CW,HIGH); // Only one of these can be high at a time
  pot.set_rdac(accel_cmd,0,0);// Set the acceleration here
  pot.write_block();

  delay(100);

  analogWrite(SpeedSet,min_speed_cmd); // Set the Speed here
  delay(1000);
  pot.set_rdac(0,0,0);// Set the acceleration here
  pot.write_block();
  gate1=1;
  //test loop
  while(true){
    delay(10);
    if (gate1==1){
      gate1=0;
      gate2=0;
      counter1=0;
      counter2=0;
      accel=accel+delta_accel;
      accel_cmd=((float)accel/2000.0)*255.0;
      //analogWrite(ACC,(int)accel_cmd);
      pot.set_rdac(255,0,0);// Set the acceleration here
      pot.write_block();
      analogWrite(SpeedSet,min_speed_cmd);
      gate3++; //this will count up until gate3 opens
      time_delay=max_speed/(float)accel*1000.0+2000.0;
      delay(time_delay);
    }

    if (gate3==(max_accel/delta_accel+1)){
      while(1){} //this will end the program once the max acceleration has been tested
    }

    Aread = analogRead(A2); //rpm of wheel
    //Serial.println(micros());
    voltageA = Aread * (5.00 / 1023.00);
    speedA = max_speed / 3.33 * (3.33-voltageA);
    Serial.println(speedA);
    counter1++;

    if (counter1==100 && gate1==0) {
      analogWrite(SpeedSet,max_speed_cmd);
    }
    if(speedA>1599.0 && gate1==0){
      gate2=1;
    }
    if(gate2==1){ //this will wait until max sp                                eed is reached and begin counter 2
      counter2++;
    }
    if(counter2==100){ //this will wait until counter 2 is at 100
      gate1=1; //opening gate 1 means doing the next acceleration loop
    }

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
      test_gyro();
      break;
    // Potentiometer only test
    case 'p':
      Serial.println("@p");
      test_pot();
      break;
      // DAC only test
    case 'd':
      Serial.println("@d");
      test_dac();
      break;
      //Motor x test
    case 'x':
      Serial.println("@x");
      test_x();
      break;
      //imu test
    case 'i':
      Serial.println("@i");
      test_imu_filter();
      break;
    // Unrecognized charactar code
    default:
      Serial.println("!Unrecognized test charactar code");
  }
}

#endif
