//
// ADCSCode/ADCSCode.ino
//
// Created by Kyle Krol (kpk63@cornell.edu) on 7/21/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include <AdafruitADC.hpp>

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

/*! Sun sensor assembly code */
namespace ssa {

/*! Number of channels read every loop (legal values are 1, 2, 3, and 4) */
#define SSA_CHANNELS_PER_READ 1

  /*! Defining the five analog to digital converters */
  ADS1015 adcs[5] = {
    ADS1015(Wire, ADS1015::ADDR::GND, 11),
    ADS1015(Wire, ADS1015::ADDR::VDD, 12),
    ADS1015(Wire, ADS1015::ADDR::SCL, 14),
    ADS1015(Wire1, ADS1015::ADDR::VDD, 13),
    ADS1015(Wire1, ADS1015::ADDR::SDA, 20)
  };

  /*! Defining the 20 dimensional vector for all the analog reads */
  int16_t data[20] = { 0 };

  /*! Defining the broken flags array (one failure counter for each ADC) */
  unsigned int broken[5] = { 0 };

  /*! Most recently read ADC channel */
  unsigned int last_channel = 0;

#ifdef TESTING
  /*! Outputs a portion of a csv line containing all 20 analog reads and the
   *  broken flags array. The following format is used:
   *    data[0],data[1],...,data[19],broken[0],...,broken[4]
   */
  void verbose_output() {
    for(unsigned int i = 0; i < 20; i++) {
      Serial.print(data[i]);
      Serial.print(',');
    }
    for(unsigned int i = 0; i < 4; i++) {
      Serial.print(broken[i]);
      Serial.print(',');
    }
    Serial.print(broken[4]);
  }
#endif

  /*! Configures and reads initial data from all of the ADCS */
  void init() {
    // Write configuration settings
    for(unsigned int i = 0; i < 5; i++) {
      adcs[i].set_sample_rate(ADS1015::SR::SPS_920);
      adcs[i].set_gain(ADS1015::GAIN::ONE);
    }
    // Read all ADC channels
    for(unsigned int i = 0; i < 3; i++) {
      for(unsigned int j = 0; j < 5; j++)
        adcs[j].start_read(i);
      for(unsigned int j = 0; j < 5; j++) {
        // Attempt first read
        if(!adcs[j].end_read(data[4 * j + i])) {
          data[4 * j + i] = 0;
          broken[j]++;
        }
      }
    }
  }

  /*! Reads the sun sensors and determines a new sun vector. The function
   *  returns true if the sun vector should be considered good data and false
   *  otherwise. The sun vector is written into the specified float array.
   */
  bool read(float *sun_vector) {
    for(unsigned int i = 0; i != (last_channel + SSA_CHANNELS_PER_READ) % 4; i = (i + 1) % 4) {
      for(unsigned int j = 0; j < 5; j++)
        adcs[j].start_read(i);
      for(unsigned int j = 0; j < 5; j++) {
        if(!adcs[j].end_read(data[4 * j + i])) {
          data[4 * j + i] = 0;
          broken[j]++;
        }
      }
    }
    // TODO : Actually determine data and its usefulness
    sun_vector[0] = 0.0f;
    sun_vector[1] = 0.0f;
    sun_vector[2] = 1.0f;
    return false;
  }

}

void setup() {
  Wire.begin();
  Wire1.begin();
#ifdef VERBOSE
  // Start serial port for verbose output
  Serial.begin(9600);
#endif
#ifdef TESTING
  // Testing relies on a user input of a test code (see switch statement)
  while(Serial.available() < 1);
  unsigned char code = Serial.read();
  switch (code) {
    case 'a': // SSA assembly test alone
      ssa::init();
      Serial.println(String(millis()) + ",");
      ssa::verbose_output();
      Serial.println();
      while(true) {
        float v[3];
        ssa::read(v);
        Serial.println(String(millis()) + ",");
        ssa::verbose_output();
        Serial.println();
        delay(20);
      }
      break;
    default:
#endif

  // TODO : Actual full flight code goes here

#ifdef TESTING
    break;
  }
#endif
}

void loop() {

}
