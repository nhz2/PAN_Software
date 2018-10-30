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
  /** \enum SR
   *  \brief Enumerates the sample rate settings for the \c ADS1015.
   *
   *  \c SPS_100 would stand for 100 samples per second. The default samples per
   *  second value for the \c ADS1015 is 1600. **/
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
  enum GAIN {
    PM_6_1444 = 0x0000,
    PM_4_096V = 0x0200,
    PM_2_048V = 0x0400,  // Default
    PM_1_024V = 0x0600,
    PM_0_512V = 0x0800,
    PM_0_256V = 0x0A00
  };

 private:
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace ADS1015_V1
#endif

#endif

/** @} **/
