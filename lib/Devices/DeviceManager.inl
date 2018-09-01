//
// Devices/DeviceManager.inl
// PAN
//
// Contributors:
//   Kyle Krol         kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_DEVICES_DEVICEMANAGER_INL_
#define PAN_DEVICES_DEVICEMANAGER_INL_

#include "DeviceManager.hpp"

namespace Devices {

namespace v1 {
DeviceManager::DeviceManager(std::vector<Device *> const &devices)
    : devices(devices) {
  // empty
}

template <class DeviceType>
inline DeviceType &DeviceManager::get_device(std::size_t i) {
  return *static_cast<DeviceType *>(this->devices[i]);
}

inline std::size_t DeviceManager::get_device_count() const {
  return this->devices.size();
}

template <class DeviceType>
inline void for_each(void (*func)(DeviceType &), std::size_t s = 0,
                     std::size_t e = this->devices.size()) {
  for (std::size_t = i, i < e; i++)
    func(*static_cast<DeviceType *>(this->devices[i]));
}
}  // namespace v1
}  // namespace Devices

#endif
