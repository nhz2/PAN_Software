//
// Devices/Device.hpp
// PAN
//
// Contributors:
//   Tanishq Aggarwal  ta335@cornell.edu
//   Kyle Krol         kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/** \addtogroup Libraries
 *  @{ **/

#ifndef PAN_DEVICES_DEVICE_HPP_
#define PAN_DEVICES_DEVICE_HPP_

/** \namespace Devices
 *  \brief Contains all of the general device functionality shared between the
 *         flight and ADCS computers.
 *
 *  This interface ensures the following functionality: a setup functions to be
 *  called on device initiation, a function which specifies whether or not the
 *  device is currently operating properly, a hard reset function, a function
 *  to disable to device, and a function to print testing data over Serial. **/
namespace Devices {
#ifndef DOXYGEN_SHOULD_SKIP_THIS
inline namespace DEVICE_V1 {
#endif
/** \interface Device
 *  \brief Interface from which all peripherial devices will be derived.
 *
 *  This interface ensures that all peripherials in communication with a
 *  flight computer have common functionality. This will be most useful for
 *  communications downlinks and updates on the satellites health. **/
class Device {
 public:
  /** \brief Sets up communication with the device and verifies
   *         the device is responding to communication attempts.
   *
   *
   *  \returns True if device is working properly, false otherwise. **/
  virtual bool setup() = 0;
  /** \brief Verifies the device is responding to communications.
   *  \returns True if device is responding to communications, false otherwise.
   * **/
  virtual bool is_functional() const = 0;
  /** \brief Attempts to reset a non-functional device. All error state
   *         variables should be reset. In most cases, this should only be
   *         be called as the result of a ground originated command. **/
  virtual void reset() = 0;
  /** \brief Disables a device regardless of it's current error state. In most
   *         cases, this should only be called as the result of a ground
   *         originated command. **/
  virtual void disable() = 0;
  /** \brief Performs the device's single component test. This test writes a csv
   *         formatted line over Serial. See the actual function implementation
   *         for more details.
   *  \returns csv file format String **/
  virtual void single_comp_test() = 0;
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace DEVICE_V1
#endif
}  // namespace Devices

#endif

/** @} **/
