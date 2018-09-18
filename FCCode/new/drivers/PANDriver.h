#ifndef PANDRIVER_H_
#define PANDRIVER_H_

#include <string>

namespace PAN {
namespace Devices {
// Interface adhered to by all PAN devices.
class PANDriver
{
public:
  /** \brief Sets up communication with the device and verifies 
     *         the device is responding to communication attempts.
     *  \returns True if device is working properly, false otherwise. **/
  virtual bool setup() = 0;
  /** \brief Verifies the device is responding to communications.
     *  \returns True if device is responding to communications, false otherwise. **/
    virtual bool is_responding() = 0;
    /** \brief Returns a string containing detailed device state.
     * Ideally this should be in CSV format.
     * For use on ground testing only.
     * \return The string of test information.
     * **/
    virtual std::string self_test() = 0;
};
} // namespace Devices
} // namespace PAN

#endif