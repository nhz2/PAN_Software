//
// ADC1015.hpp
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_ADC1015_HPP_
#define PAN_ADC1015_HPP_

#include <I2CDevice.hpp>

/** Possible address values for the ADS1015. **/
enum ADS1015_ADDR { GND = 0x48, VDD = 0x49, SDA = 0x4A, SCL = 0x4B };

/** Possible gain values for the ADS1015. **/
enum ADS1015_GAIN {
  TWO_THIRDS = 0x0000,
  ONE = 0x0200,
  TWO = 0x0400,  // Default
  FOUR = 0x0500,
  EIGHT = 0x0800,
  SIXTEEN = 0x0A00
};

/** Possible sample rates for the ADS1015. **/
enum ADS1015_SR {
  SPS_128 = 0x0000,
  SPS_250 = 0x0020,
  SPS_490 = 0x0040,
  SPS_920 = 0x0060,
  SPS_1600 = 0x0080,  // Default
  SPS_2400 = 0x00A0,
  SPS_3300 = 0x00C0
};

inline namespace ADS1015_V1 {
/** Class representing a ADS1015 IC. Please note their are only methods to read
 *  the ADC in a single ended manner and that you must be using the alrt/ready
 *  pin on the device. **/
class ADC1015 : public Devices::I2CDevice {
 public:
  /** \brief Setup function that ensures the device is responding properly. This
   *         function queries that value of the three lest significant bits in
   *         the conversion registers which are always zero. The function fails
   *         an i2c response is not recieved or if those bits don't read zero.
   *  \returns true if device is working properly, false otherwise **/
  virtual bool setup() override;
  /** \brief Formats a data string for the ADS1015. Intended to be used for
   *         testing purposes. The data line has the following format:
   *           v0, v1, v2, v3
   *  \returns the data string **/
  virtual std::string get_data_string() const;
  /** \brief Attempts to reset the device by calling the I2CDevice cmd_reset
   *         function and then resets the device. **/
  virtual void cmd_reset() override;
  /** \brief Sets up and ADC1015 IC on the specified wire with the given address
   *         and alert pin. **/
  ADS1015(i2c_t3 &wire, ADS1015_ADDR addr, unsigned int alert_pin);
  /** \brief Sets the gain value of the ADS1015. **/
  inline void set_gain(ADS1015_GAIN gain) { this->gain = gain; }
  /** \brief Returns the current gain of the ADS1015.
   *  \returns current gain value **/
  inline ADS1015_GAIN get_gain() const { return this->gain; }
  /** \brief Sets the sample rate of the ADS1015 **/
  void set_sample_rate(ADS1015_SR sample_rate);
  /** \brief Returns the current sample rate of the ADS1015.
   *  \returns current sample rate value **/
  ADS1015_SR get_sample_rate() const { return this->sample_rate; }
  /** \brief Starts a conversion on the ADC1015 on the specified channel (0, 1,
   *         2, and 3 are valid inputs). The conversion can be completed with a
   *         call to end_read. This is a non-blocking call - you may even use
   *         other devices on the same wire. **/
  void start_read(unsigned int channel);
  /** \brief Ends the conversion that was previously started with a call to
   *         start_read. The recorded value is returned. Check i2c_data_is_valid
   *         to ensure data integrity.
   *  \returns 12 bit converted value **/
  int16_t end_read();
  /** \brief Equivalent to consecutive calls to start_read and end_read on the
   *         specified channel. Check i2c_data_is_valid to ensure data
   *         integrity.
   *  \returns 12 bit converted value **/
  int16_t read(unsigned int channel);

 private:
  /** ADC conversion complete alert pin **/
  unsigned int const alert_pin;
  /** Timestamp of the start of the most recent conversion **/
  unsigned long timestamp;
  /** Sample timeout delay **/
  unsigned int sample_delay;
  /** Alert pin configuration needed flag **/
  bool alert_config_needed;
  /** ADC samples per second **/
  ADS1015_SR sample_rate;
  /** ADC gain value **/
  ADS1015_GAIN gain;
};
}  // namespace ADS1015_V1

#endif
