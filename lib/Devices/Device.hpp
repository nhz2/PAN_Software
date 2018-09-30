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
  /** \brief Initiliazes the device and attempts communication if applicable.
   *  \returns true if initialization succeeded and false otherwise.
   *
   *  The setup function is used to initialize a device when the satellite is
   *  booting up. The function should perform as many of the following functions
   *  that are applicable to a particular device:
   *   -# Set the device's input values to defaults to avoid erroneous behavior.
   *      A good example of this would be disable CW and CCW rotation of the
   *      reaction wheels.
   *   -# "Ping" a device that we are communicating with over I2C/UART to ensure
   *      the device is responding. Reading a dummy register off of and i2c
   *      device or sending the "AT" command to the quake are good examples.
   *  If the device doesn't fit any of the above criteria, it is permissable for
   *  the function to just return true and act as a NOP. **/
  virtual bool setup();
  /** \brief Verifies the device is responding to communications.
   *  \returns True if device is responding to communications, false otherwise.
   *
   *  This functions simply returns whether or not the current device should be
   *  treated as functional. A device may be marked not functional for two
   *  reasons:
   *   -# The device was disabled by a call to the disable function.
   *   -# The device marked itself as not functional do to multiple,
   *      consecutive communication failures.
   *  Communication with the device should not be attempted if it is marked
   *  as not functional. **/
  virtual bool is_functional() const;
  /** \brief Attempts to reset a non-functional device. All error state
   *         variables should be reset. In most cases, this should only be
   *         be called as the result of a ground originated command. **/
  virtual void reset();
  /** \brief Disables a device regardless of it's current error state. In most
   *         cases, this should only be called as the result of a ground
   *         originated command. **/
  virtual void disable();
  /** \brief Performs the device's single component test. This test writes a csv
   *         formatted line over Serial. See the actual function implementation
   *         for more details.
   *  \returns csv file format String **/
  virtual void single_comp_test();
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace DEVICE_V1
#endif
}  // namespace Devices

#endif

/** @} **/
