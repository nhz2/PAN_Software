//
// Devices/DeviceManager.hpp
// PAN
//
// Contributors:
//   Kyle Krol         kpk63@cornell.edu
//   Tanishq Aggarwal  ta335@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/** \addtogroup Libraries
 *  @{ **/

#ifndef PAN_DEVICES_DEVICEMANAGER_HPP_
#define PAN_DEVICES_DEVICEMANAGER_HPP_

#include "Device.hpp"

#include <cstddef>

namespace Devices {
#ifndef DOXYGEN_SHOULD_SKIP_THIS
inline namespace DEVICEMANAGER_V1 {
#endif
/** \class DeviceManager
 *  \brief Serves as a single location where references to all of the
 *         microcontrollers devices are listed.
 *
 *  The DeviceManager class is intended to serve as a single location where
 *  references to all of a microcontrollers devices are listed. This is
 *  convenient for high level functionality like comms frames, device enabling &
 *  disabling, etc.
 *  The intended use case is as follows:
 *   -# Create a constant array of pointers to all of the devices conencted to
 *     the microcontroller.
 *   -# Enum the device identifiers (position in the vector) for easy, efficient
 *     access - this also makes static casting via the get_device function
 *     easier to perform.
 *   -# Pass a reference of the device pointer vector to the DeviceManager
 *     constructor. **/
class DeviceManager {
 public:
  /** \brief Constructs a new device manager controlling the specified devices.
   *  \param[in] devices Pointer to the device array.
   *  \param[in] len Length of the device array.
   *
   *  The device manager should be constructed after a constant array of Device
   *  pointers to all of the microcontrollers devices has been created. See the
   *  DeviceManager class description for more information . **/
  DeviceManager(Device *const *const devices, std::size_t len);
  /** \brief Returns the requested device statically casted to the requested
   *         type.
   *  \tparam DeviceType Type that the requested device will be statically
   *          casted to.
   *  \param[in] i Index of the desired device in the backing array.
   *  \returns A statically casted reference to the desired Device.
   *
   *  The validity of the static cast should be determined at compile time -
   *  i.e. it won't cause any errors at run time and there is virtually no
   *  overhead. **/
  template <class DeviceType = Device>
  inline DeviceType &get_device(std::size_t i);
  /** \brief Returns the size of the Device pointer backing array.
   *  \returns Length of the backing array. **/
  inline std::size_t get_device_count() const;
  /** \brief Executes a function on a range of devices in the Device pointer
   *         backing array.
   *  \tparam DeviceType Type of device the pointers will be statically casted
   *          to.
   *  \param[in] func Function to be performed on the devices.
   *  \param[in] s Inclusive lower bound on the devices being operated on.
   *  \param[in] e Exclusive upper bound on the devices being operated on.
   *
   *  It essentially acts like a for each call on a subset of the Device pointer
   *  backing array. The function func will be applied to a statically casted
   *  reference of every Device within the range [s,e) of the backing array. **/
  template <class DeviceType = Device>
  inline void for_each(void (*func)(DeviceType &), std::size_t s = 0,
                       std::size_t e = this->len);

 protected:
  /** \brief Device pointer backing array **/
  Device *const *const devices;
  /** \brief Length of the device pointer backing array. **/
  std::size_t const len;
};
#ifndef DOXYGEN_SHOULD_SKIP_THIS
}  // namespace DEVICEMANAGER_V1
#endif
}  // namespace Devices

#include "DeviceManager.inl"

#endif

/** @} **/
