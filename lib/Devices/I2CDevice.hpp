//
// Devices/I2CDevice.hpp
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
 *  @{ **/

#ifndef PAN_DEVICES_I2CDEVICE_HPP_
#define PAN_DEVICES_I2CDEVICE_HPP_

#include "Device.hpp"

/** Note that I2C_AUTO_RETRY should be enabled - I2CDevice makes no calls to
 *  resetBus internally. **/
#include "i2c_t3_pan/i2c_t3_pan.h"

/** \namespace Devices **/
namespace Devices {

/** The number of times an i2c communication can fail before the device is
 *  considered not functional. **/
#define I2CDEVICE_DISABLE_AT 3

#ifndef DOXYGEN_SHOULD_SKIP_THIS
inline namespace I2CDEVICE_V1 {
#endif
/** \class I2CDevice
 *  \brief Abstract class from which all i2c device drivers will be derived.
 *
 *  \par
 *  The I2CDevice class is intended as an abstract class form which all i2c
 *  devices will be derived.
 *  \par
 *  It provides per device i2c fucntionality as opposed to per bus making code
 *  and device managment much simpler. Each device specifically has it's own
 *  timeout value in milliseconds, i2c bus address, i2c bus reference, and error
 *  history state variables.
 *  \par
 *  The class also wraps common i2c_t3 wire function calls to manage the device
 *  device specific variables (error tracking, device specific timeouts, etc.).
 *  See https://github.com/nox771/i2c_t3 for more information about the teensy
 *  i2c library.
 *  \par
 *  The error history state variables operate in the following manner. Each
 *  device has two variables tracking errors. One tracks recent history and is a
 *  boolean that holds true if an error has occurred since the last call to
 *  i2c_pop_errors() and holds false otherwise. The second variable counts
 *  consecutive communication failures with the device and is updated with a
 *  call to i2c_pop_errors(). i2c_pop_errors should therefore only be called
 *  once during a communication with an i2c device (this includes configuration,
 *  requesting, and recieving that is needed to obtain data). Otherwise,
 *  i2c_peek_errors() can be used to check the recent history of i2c errors.
 *  \par
 *  For consistency, all member functions of this class are prefixed with i2c_.
 *  **/
class I2CDevice : public Device {
 public:
  /** \brief Attempts to initialize communication with the i2c device.
   *  \returns True if the setup was succesful and false otherwise.
   *
   *  The function makes up to I2CDEVICE_DISABLE_AT calls to the i2c_ping
   *  function. If any of those calls returns true, the setup function is
   *  considered to succesful. If the setup function returns false, this implies
   *  the device is not functional and is marked as such. **/
  virtual bool setup() override;

  virtual bool is_functional() const override;
  /** \brief Forces this i2c device to be marked as functional.
   *
   *  The error history variable is reset which therefore marks the device as
   *  functional. The device will still mark itself as not functional if it has
   *  I2CDEVICE_DISABLE_AT i2c errors in a row. **/
  virtual void reset() override;
  /** \brief Forces this i2c device to be marked as not functional.
   *
   *  The error history variable is set to I2CDEVICE_DISABLE_AT which causes the
   *  i2c device to be seen as not functional. This can be reversed by calling
   *  the reset function. **/
  virtual void disable() override;
  /** \brief Sets this device's i2c timeout in milliseconds.
   *  \param[in] i2c_timeout New i2c timeout in milliseconds. **/
  inline void i2c_set_timeout(unsigned long i2c_timeout);
  /** \brief Gets the current value of i2c_timeout in milliseconds.
   *  \return current i2c_timeout in milliseconds. **/
  inline unsigned long i2c_get_timeout() const;
  /** \brief Specifies whether the most recent i2c data should be considered
   *         valid.
   *  \returns True if the most recent data is valid and false otherwise.
   *
   *  The data is considered valid if the error history variable currently has
   *  a value of zero - i.e. the last i2c communication did not result in an
   *  error. **/
  inline bool i2c_data_is_valid() const;

 protected:
  /** \brief Attempts a simple communication with the i2c device - e.g. reading
   *         a register with a known value. Returns true if the proper value is
   *         recieved. i2c related errors are queried in setup.
   *  \returns true if the proper value was read and false otherwise **/
  virtual bool i2c_ping();
  /** \brief Constructs a new i2c device object.
   *  \param[in] wire Reference to the Wire this i2c device is on.
   *  \param[in] addr Address byte of this i2c device.
   *  \param[in] timeout Timeout value fro this i2c device in milliseconds. **/
  I2CDevice(i2c_t3 &wire, uint8_t addr, unsigned long timeout = 0);
  /** \brief Returns true if an error has occurred since the last call to
   *         pop_errors and false otherwise. The recent error history variable
   *         is reset and the consecutive communication failure variables is
   *         incremented if needed.
   *  \returns true if an error has occurred since the last call to pop_errors
   *           and false otherwise. **/
  inline bool i2c_pop_errors();
  /** \brief Identical functionality to pop_errors except the recent error
   *         history variable isn't reset and the consecutive communication
   *         failure variable isn't incrememnted.
   *  \returns true if an error has occurred since the last call to pop_errors
   *           and false otherwise. **/
  inline bool i2c_peek_errors() const;
  /** \brief Sends a message over i2c to this i2c device.
   *  \tparam Type of data written to the outgoing i2c buffer.
   *  \param[in] data Data to be written to the outgoing i2c buffer.
   *  \param[in] len Number of elements to be written from data.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  This performs a series of blocking i2c commands to send data to this i2c
   *  device. Any error is recorded in the recent error history variable. **/
  template <typename T>
  void i2c_transmit_data(T const *data, std::size_t len, i2c_stop s = I2C_STOP);
  /** \brief Requests and recieves a message over i2c from this i2c device.
   *  \tparam T Type of data read form the incoming i2c buffer.
   *  \param[out] data Data buffer data will be written into.
   *  \param[in] len Length of the data buffer to be overwritten.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  This performs a series of blocking i2c commands to request data from this
   *  i2c device, read it into the the supplied data buffer, and update the
   *  recent error history variable. Be sure to check for data integrity with a
   *  call to i2c_data_is_valid. **/
  template <typename T>
  void i2c_receive_data(T *data, std::size_t len, i2c_stop s = I2C_STOP);
  /** \brief Inlined call to Wire.beginTransmission in i2c_t3.
   *
   *  The i2c device's address is passed as an argument to the call to
   *  Wire.beingTransmission. **/
  inline void i2c_begin_transmission();
  /** \brief Inlined call to Wire.endTransmission from i2c_t3.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  The provided or default i2c stop flag as well as this i2c devices timeout
   *  value are passed to Wire.endTransmission. Errors reported by
   *  Wire.endTransmission are recorded in the recent error history variable.
   * **/
  inline void i2c_end_transmission(i2c_stop s = I2C_STOP);
  /** \brief Inlined call to Wire.sendTransmission from i2c_t3.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  Only the i2c stop flag is passed to the call to Wire.sendTransmission
   *  given it is a non-blocking call. **/
  inline void i2c_send_transmission(i2c_stop s = I2C_STOP);
  /** \brief Inlined call to Wire.requestFrom in i2c_t3.
   *  \param[in] len Length of message in bytes requested.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  The requested length, i2c stop flag, i2c device address, and i2c device
   *  timeout are all passed to the call to Wire.requestFrom. If the proper
   *  number of bytes is not recieved, an error is recorded in the recent error
   *  history variable. **/
  inline void i2c_request_from(std::size_t len, i2c_stop s = I2C_STOP);
  /** \brief See Wire.sendRequest in i2c_t3. Any error will be recorded in the
   *         recent error history variable. **/

  /** \brief Inlined call to Wire.sendRequest from i2c_t3.
   *  \param[in] len Length of message in bytes requested.
   *  \param[in] s Sets whether or not an i2c stop signal is sent at the end of
   *               the transmission.
   *
   *  The requested length, i2c stop flag, and i2c device address are all passed
   *  to the call to Wire.sendRequest. The i2c device's timeout is not because
   *  this call is non-blocking. **/
  inline void i2c_send_request(std::size_t len, i2c_stop s = I2C_STOP);
  /** \brief Inlined call to Wire.done from i2c_t3.
   *  \returns false if a non-blockign call is still running and true otherwise.
   * **/
  inline bool i2c_done() const;
  /** \brief Inlined call to Wire.finish from i2c_t3.
   *
   *  Any error is recorded in the recent error history variable. **/
  inline void i2c_finish();
  /** \brief See Wire.write in i2c_t3. Any error will be recorded in the recent
   *         error history variable. **/

  /** \brief Inline call to Wire.write from i2c_t3.
   *
   *  Writes a single byte to the outgoing i2c buffer. Any error is recorded in
   *  the recent error history variable. **/
  inline void i2c_write(uint8_t data);
  /** \brief Data type templated inlined call to Wire.write from i2c_t3.
   *  \tparam T Type of data written to the i2c outgoing buffer.
   *  \param[in] data Data array to be written to the i2c outgoing buffer.
   *  \param[in] len Length of the data array.
   *
   *  Writes an array of arbitrary data types to the outgoing i2c buffer. Any
   *  error is recorded in the recent error history variable. **/
  template <typename T>
  inline void i2c_write(T const *data, std::size_t len);
  /** \brief See Wire.readByte in i2c_t3. Any error will be recorded in the
   *         recent error history variable.
   *  \returns next uint8_t in the incoming i2c buffer. **/

  /** \brief Inlined call to Wire.readByte from i2c_t3.
   *  \returns next uint8_t in the incoming i2c buffer.
   *
   *  Any error is recorded in the recent error history variable. Be sure to
   *  check for data integrity with a call to i2c_data_is_valid. **/
  inline uint8_t i2c_read();
  /** \brief Data type tmeplated inlined call to Wire.read from i2c_t3.
   *  \tparam T Type of data read from the i2c incoming buffer.
   *  \param[out] data Data array to be written to from the i2c incoming buffer.
   *  \param[in] len Number of elements to be written over in the data array.
   *
   *  Fills an array of arbitrary data type with data read from the incoming
   *  i2c buffer. Any error is recorded in the recent error history variable. Be
   *  sure to check for data integrity with a call to i2c_data_is_valid. **/
  template <typename T>
  inline void i2c_read(T *data, std::size_t len);

 private:
  /** Wire associated with this device **/
  i2c_t3 &wire;
  /** I2C address associated with this device **/
  uint8_t const addr;
  /** Timeout value associated with this device for wire calls **/
  unsigned long timeout;
  /** Keeps track of consecutive communication errors **/
  unsigned int error_count;
  /** Error history tracker **/
  bool recent_errors;
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace I2CDEVICE_V1
#endif
}  // namespace Devices

#include "I2CDevice.inl"

#endif

/** @} **/
