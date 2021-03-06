//
// ADCSCode/ssa.hpp
//
// Contributors:
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/* This header file contains function declarations for the sun sensor assembly
 * incorporated into the ADCS system. The header is designed to be used with
 * the TESTING and VERBOSE macros as defined in ADCSCode/ADCSCode.ino.
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

/*! How many sun sensor ADC units their are */
#define SSA_ADC_COUNT 5

/*! Namespace for sun sensor assembly related things */
namespace ssa {

  /*! Defining the five analog to digital converters. The address values are
   *  subject to change.
   */
  extern ADS1015 adcs[SSA_ADC_COUNT];

  /*! 20 entry 16 bit signed integer array to store raw ADC reads */
  extern int16_t raw_data[SSA_ADC_COUNT * 4];

  /*! Tracks consecutive failed I2C communications on behalf of each ADC - i.e.
   *  once a proper communication is made, it's broken value is set back to 0.
   *  An entire ADC will be ignored if it fails SSA_IGNORE_ON_READS consecutive
   *  communication attempts.
   */
  extern unsigned int consec_err[SSA_ADC_COUNT];

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
   *    raw_data[0],raw_data[1],...,raw_data[19],consec_err[0],...,consec_err[4]
   */
  void verbose_output() {
    for(unsigned int i = 0; i < SSA_ADC_COUNT * 4; i++) {
      Serial.print(raw_data[i]);
      Serial.print(',');
    }
    for(unsigned int i = 0; i < SSA_ADC_COUNT; i++) {
      Serial.print(consec_err[i]);
      Serial.print(',');
    }
    Serial.print(consec_err[4]);
  }

  /*! Reports any ADC communication error that has occurred - i.e. the
   *  consec_err value is non-zero. This methods prints lines with the "!"
   *  alert modifier.
   */
  void verbose_error() {
    for(unsigned int i = 0; i < SSA_ADC_COUNT; i++)
      if(consec_err[i] > 0)
        Serial.println("!ADC " + String(i) + " had a communication error");
  }
#endif

}

#endif
