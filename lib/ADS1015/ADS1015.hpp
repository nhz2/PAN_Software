//
// ADS1015.hpp
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/** \addtogroup Libraries
 * @{ **/

#ifndef PAN_ADS1015_HPP_
#define PAN_ADS1015_HPP_

#include "../Devices/I2CDevice.hpp"

#ifndef DOXYGEN_SHOULD_SKIP_THIS
inline namespace ADS1015_V1 {
#endif
/** \class ADS1015
 *  \brief Driver for Texas Instruments ADS1015 12 bit ADC.
 *
 *  This driver is explicitly written to be used with the ADC placed in single
 *  shot conversion mode while utilizing the alert/ready pin to signal for
 *  the completion of a conversion. The alert/ready pin of the device must be an
 *  input to a GPIO pin. The driver was based on code from this repository:
 *  https://github.com/adafruit/Adafruit_ADS1X15. **/
class ADS1015 : public Devices::I2CDevice {
 public:
  /** \enum ADDR
   *  \brief Possible i2c addresses for the \c ADS1015. **/
  enum ADDR { GND = 0x48, VDD = 0x49, SDA = 0x4A, SCL = 0x4B };
  /** \enum SR
   *  \brief Enumerates the sample rate settings for the \c ADS1015.
   *
   *  \c SPS_100 would stand for 100 samples per second. The default samples per
   *  second value for the \c ADS1015 is \c SPS_1600. **/
  enum SR : uint16_t {
    /** \brief 128 samples per second. **/
    SPS_128 = 0x0000,
    /** \brief 250 samples per second. **/
    SPS_250 = 0x0020,
    /** \brief 490 samples per second. **/
    SPS_490 = 0x0040,
    /** \brief 920 samples per second. **/
    SPS_920 = 0x0060,
    /** \brief 1600 samples per second. **/
    SPS_1600 = 0x0080,
    /** \brief 2400 samples per second. **/
    SPS_2400 = 0x00A0,
    /** \brief 3300 samples per second. **/
    SPS_3300 = 0x00C0
  };
  /** \enum GAIN
   *  \brief Enumerates the gain settings for the \c ADS1015.
   *
   *  The default gain value for the \c ADS1015 is \c PM_2_048V. **/
  enum GAIN {  // TODO
    PM_6_1444 = 0x0000,
    PM_4_096V = 0x0200,
    PM_2_048V = 0x0400,
    PM_1_024V = 0x0600,
    PM_0_512V = 0x0800,
    PM_0_256V = 0x0A00
  };
  /** \brief Outputs the result of an analog read of all four channels of the
   *         \c ADS1015.
   *
   *  Requires that \c Serial has been initialized. **/
  virtual void single_comp_test();
  /** **/  // TODO
  virtual bool i2c_ping();
  /** \brief Creates an \c ADS1015 object communicating on the specified i2c bus
   *         with the given alert gpio and timeout value.
   *  \param[in] i2c_wire i2c bus used to communicate with the device.
   *  \param[in] i2c_addr i2c address of this device.
   *  \param[in] i2c_timeout Sets the this devices i2c timeout in milliseconds.
   *  \param[in] alert_pin GPIO pin connected to this device's alert output.
   *
   *  It is assumed that the specified wire was already initialized and the pin
   *  mode of \c alert_pin was previously set to \INPUT. **/
  ADS1015(i2c_t3 &i2c_wire, ADDR i2c_addr, unsigned long timeout,
          unsigned int alert_pin);
  /** \brief Sets the gain of the \c ADS1015. **/
  inline void set_gain(GAIN gain) { this->gain = gain; }
  /** \brief Returns the current gain setting of the \c ADS1015.
   *  \returns Current gain setting. **/
  inline GAIN get_gain() const { return this->gain; }
  /** \brief Sets the sample rate of the \c ADS1015. **/
  void set_sample_rate(SR sample_rate);
  /** \brief Returns the current sample rate of the \c ADS1015.
   *  \returns Current sample rate. **/
  inline SR get_sample_rate() const { return this->sample_rate; }
  /** \brief Starts an analog conversion on the specified channel.
   *  \param[in] channel Channel on which the analog read is started.
   *
   *  The result of conversion is obtained by making a call to \c end_read at
   *  some unspecified time later on. **/
  void start_read(unsigned int channel);
  /** \brief Obtains the result of an analog conversion. \c start_read must have
   *         been called.
   *  \returns The result of the analog conversion.
   *
   *  Note, that to ensure to data's integrity a call to \c i2c_pop_errors
   *  should be made. **/
  int16_t end_read();
  /** \brief Performs an analog converison and returns the result.
   *  \param[in] channel Channel on which the analog read is started.
   *  \returns The result of the analog conversion.
   *
   *  Essentially, this function is equivalent to making a call to \c start_read
   *  and then \c end_read. Data integrity needs to be ensured by making a call
   *  to \c i2c_pop_errors. **/
  int16_t read(unsigned int channel);

 private:
  /** \brief \c ADS1015 alert pin number. */
  unsigned int const alert_pin;
  /** \brief Previous conversion start timestamp. */
  unsigned long timestamp;
  /** \brief Current sample timeout delay in milliseconds. */
  unsigned int sample_delay;
  /** \brief Signals the \c ADS1015 needs to be configured prior to reading. */
  bool alert_config_needed;
  /** \brief Current sample rate setting. */
  SR sample_rate;
  /** \brief Current gain setting. */
  GAIN gain;
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace ADS1015_V1
#endif

#endif

/** @} **/
