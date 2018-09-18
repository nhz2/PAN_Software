#ifndef PANDRIVER_H_
#define PANDRIVER_H_

namespace PAN
{
namespace Devices
{
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
};
} // namespace Devices
} // namespace PAN

#endif