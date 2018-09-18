//
// libraries/AdafruitADC/AdafruitADC.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#ifndef LIBRARIES_ADAFRUITADC_ADAFRUITADC_HPP
#define LIBRARIES_ADAFRUITADC_ADAFRUITADC_HPP

#include <I2CDevice.hpp>

/*! This library is intended to be used with the Texas Instruments ADS1115
 *  analog to digital converter. This specific class uses the ADC's conversion
 *  ready functionality of the IC's alert/ready pin. This ensures minimum
 *  conversion delay for all sample rates. Varaible sample rates and gain values
 *  are supported.
 */
class ADS1015 : public I2CDevice {
 public:
  /*! Possible i2c addresses intended to be passed to the ADS1115 constructor */
  enum ADDR { GND = 0x48, VDD = 0x49, SDA = 0x4A, SCL = 0x4B };

  /*! Possible gain values to be passed to set_gain */
  enum GAIN {
    TWO_THIRDS = 0x0000,
    ONE = 0x0200,
    TWO = 0x0400,  // Default
    FOUR = 0x0500,
    EIGHT = 0x0800,
    SIXTEEN = 0x0A00
  };

  /*! Possible sample rates to be passed to set_sample_rate */
  enum SR {
    SPS_128 = 0x0000,
    SPS_250 = 0x0020,
    SPS_490 = 0x0040,
    SPS_920 = 0x0060,
    SPS_1600 = 0x0080,  // Default
    SPS_2400 = 0x00A0,
    SPS_3300 = 0x00C0
  };

  /*! Creates an ADC with the specified i2c bux, address, and alert pin. The
   *  sample rate and gain are set to their default values. The defaults are
   *  128 sps and gain two respectively.
   */
  ADS1015(i2c_t3 &i2c_wire, ADDR i2c_addr, unsigned int alert_pin);

  /*! Get and set the ADC gain value */
  inline void set_gain(GAIN gain) { this->gain = gain; }
  inline GAIN get_gain() const { return this->gain; }

  /*! Get and set the sample rate */
  void set_sample_rate(SR sample_rate);
  inline SR get_sample_rate() const { return this->sample_rate; }

  /*! Two stage single channel read from the ADC. Multiple conversions cannnot
   *  be running on a single device at one time. The second function waits for
   *  the alert pin to be asserted back to high to read in the conversion value.
   */
  void start_read(unsigned int line);
  bool end_read(int16_t &val);

  /*! Equivalent to sequential calls to the two stage single channel read
   *  functions.
   */
  bool read(unsigned int line, int16_t &val);

 private:
  /*! ADC conversion complete alert pin */
  unsigned int const alert_pin;

  /*! Conversion timestamp in milliseconds */
  unsigned long timestamp;

  /*! Sample timeout delay */
  unsigned int sample_delay;

  /*! Alert pin configuration needed flag */
  bool alert_config_needed;

  /*! ADC samples per second */
  SR sample_rate;

  /*! ADC gain value */
  GAIN gain;
};

#endif
