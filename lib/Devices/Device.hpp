//
// Devices/DeviceTable.hpp
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

#ifndef PAN_DEVICES_DEVICE_HPP_
#define PAN_DEVICES_DEVICE_HPP_

namespace Devices {

inline namespace DEVICE_V1 {
/** Interface adhered to by all devices. **/
class Device {
 public:
  /** \brief Sets up communication with the device and verifies
   *         the device is responding to communication attempts.
   *  \returns True if device is working properly, false otherwise. **/
  virtual bool dev_setup() = 0;
  /** \brief Verifies the device is responding to communications.
   *  \returns True if device is responding to communications, false otherwise.
   * **/
  virtual bool dev_is_functional() const = 0;
  /** \brief Attempts to reset a non-functional device. All error state
   *         variables should be reset. In most cases, this should only be
   *         be called as the result of a ground originated command. **/
  virtual void dev_reset() = 0;
  /** \brief Disables a device regardless of it's current error state. In most
   *         cases, this should only be called as the result of a ground
   *         originated command. **/
  virtual void dev_disable() = 0;
  /** \brief Performs the device's single component test. This test outputs a
   *         String in a csv file format. See the actual function implementation
   *         for more details.
   *  \returns csv file format String **/
  virtual String dev_sc_test() = 0;
};
}  // namespace DEVICE_V1
}  // namespace Devices

#endif