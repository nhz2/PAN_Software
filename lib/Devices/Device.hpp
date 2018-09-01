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

#include <string>

namespace Devices {

inline namespace v1 {
/** Interface adhered to by all devices. **/
class Device {
 public:
  /** \brief Sets up communication with the device and verifies
   *         the device is responding to communication attempts.
   *  \returns True if device is working properly, false otherwise. **/
  virtual bool setup() = 0;
  /** \brief Verifies the device is responding to communications.
   *  \returns True if device is responding to communications, false otherwise.
   * **/
  virtual bool is_functional() const = 0;
  /** \brief Returns a string representing relavent device data in csv format.
   *         See the derived class for more fomatting details.
   *  \returns CSV data line string **/
  virtual std::string get_data_string() = 0;
  /** \brief Attempts to reset a non-functional device. All error state
   *         variables should be reset. In most cases, this should only be
   *         be called as the result of a ground originated command. **/
  virtual void cmd_reset() = 0;
  /** \brief Disables a device regardless of it's current error state. In most
   *         cases, this should only be called as the result of a ground
   *         originated command. **/
  virtual void cmd_disable() = 0;
};
}  // namespace v1
}  // namespace Devices

#endif
