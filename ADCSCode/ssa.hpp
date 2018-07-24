//
// ADCSCode/ssa.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 7/24/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/* This header file contains function declarations for the sun sensor assembly
 * incorporated into the ADCS system. The header is designed to be used with
 * the TESTING and VERBOSE macros as defined in ADCSCode/ADCSCode.ino. See
 * ssa.cpp for the function implementations.
 */

#ifndef ADCSCODE_SSA_HPP
#define ADCSCODE_SSA_HPP

#include <AdafruitADC.hpp>

/*! How many times a sensor must fail before ignored */
#define SSA_IGNORE_ON_READS 100

/*! Sample rate used by the ADCs. This will greatly effect the runtime of the
 *  init and read functions.
 */
#define SSA_ADCS_SAMPLE_RATE ADS1015::SR::SPS_920

/*! Namespace for sun sensor assembly related things */
namespace ssa {

  /*! Defining the five analog to digital converters. The address values are
   *  subject to change.
   */
  ADS1015 adcs[5] = {
    ADS1015(Wire,  ADS1015::ADDR::GND, 11),
    ADS1015(Wire,  ADS1015::ADDR::VDD, 12),
    ADS1015(Wire,  ADS1015::ADDR::SCL, 14),
    ADS1015(Wire1, ADS1015::ADDR::VDD, 13),
    ADS1015(Wire1, ADS1015::ADDR::SDA, 20)
  };

  /*! 20 entry 16 bit signed integer array to store raw ADC reads */
  int16_t raw_data[20] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  /*! Tracks consecutive failed I2C communications on behalf of each ADC - i.e.
   *  once a proper communication is made, it's broken value is set back to 0.
   *  An entire ADC will be ignored if it fails SSA_IGNORE_ON_READS consecutive
   *  communication attempts.
   */
  unsigned int consec_err[5] = {
    0, 0, 0, 0, 0
  };

  /*! Initiates communication with the ADCs and takes initial data on all
   *  channels. If an ADC communication fails, it's consec_err count is
   *  incremented accordingly and the corresponding sun sensor reads are set to
   *  zero.
   */
  void init();

  /*! Reads the next channel from each ADC and determines the sun vector with
   *  the new data. The 3 dimensional vector is returned in the vec array - so
   *  it must be at least three long. The method returns true or false depending
   *  on whether the sun vector is determined to be "good" or "bad" data
   *  respectively.
   */
  bool read(float *sun_vector);
  // TODO : Better define "good" and "bad" data
  // TODO : Perhaps incorporate a vector object rather than an array

#ifdef VERBOSE
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

}

#endif
