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

/*! Possible i2c addresses intended to be passed to the ADS1115 constructor */
static const uint8_t ADS1115_ADDR_GND = 0x48;
static const uint8_t ADS1115_ADDR_VDD = 0x49;
static const uint8_t ADS1115_ADDR_SDA = 0x4A;
static const uint8_t ADS1115_ADDR_SCL = 0x4B;

/*! Possible gain values to be passed to set_gain */
static const uint16_t ADS1115_GAIN_TWO_THIRDS = 0x0000;
static const uint16_t ADS1115_GAIN_ONE        = 0x0200;
static const uint16_t ADS1115_GAIN_TWO        = 0x0400;
static const uint16_t ADS1115_GAIN_FOUR       = 0x0500;
static const uint16_t ADS1115_GAIN_EIGHT      = 0x0800;
static const uint16_t ADS1115_GAIN_SIXTEEN    = 0x0A00;

/*! Possible sample rates to be passed to set_sample_rate */
static const uint16_t ADS1115_8_SPS   = 0x0000;
static const uint16_t ADS1115_16_SPS  = 0x0020;
static const uint16_t ADS1115_32_SPS  = 0x0040;
static const uint16_t ADS1115_64_SPS  = 0x0060;
static const uint16_t ADS1115_128_SPS = 0x0080;
static const uint16_t ADS1115_250_SPS = 0x00A0;
static const uint16_t ADS1115_475_SPS = 0x00C0;
static const uint16_t ADS1115_860_SPS = 0x00E0;

/*! This library is intended to be used with the Texas Instruments ADS1115
 *  analog to digital converter. This specific class uses the ADC's conversion
 *  ready functionality of the IC's alert/ready pin. This ensures minimum
 *  conversion delay for all sample rates. Varaible sample rates and gain values
 *  are supported.
 */
class ADS1115 : public I2CDevice {

public:

  /*! Creates an ADC with the specified i2c bux, address, and alert pin. The
   *  sample rate and gain are set to their default values. The defaults are
   *  128 sps and gain two respectively.
   */
  ADS1115(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned int alert_pin);

  /*! Set and return the gain value */
  void set_gain(uint16_t gain);
  uint16_t get_gain() const;

  /*! Set and return the sample rate */
  void set_sample_rate(uint16_t sample_rate);
  uint16_t get_sample_rate() const;

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
  uint16_t sample_rate;

  /*! ADC gain value */
  uint16_t gain;

};

#endif
