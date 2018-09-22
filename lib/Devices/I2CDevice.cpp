//
// Devices/I2CDevice.cpp
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "I2CDevice.hpp"

using namespace Devices;

bool I2CDevice::setup() {
  for (unsigned int i = 0; i < I2CDEVICE_DISABLE_AT; i++)
    if (this->i2c_ping()) return true;
  return false;
}

bool I2CDevice::is_functional() {
  return (this->error_count < I2CDEVICE_DISABLE_AT);
}

void I2CDevice::reset() {
  this->error_count = 0;
  this->recent_errors = false;
}

void I2CDevice::disable() {
  this->error_count = I2CDEVICE_DISABLE_AT;
  this->recent_errors = true;
}

I2CDevice::I2CDevice(i2c_t3 &wire, uint8_t addr, unsigned long timeout)
    : wire(wire),
      addr(addr),
      timeout(timeout),
      error_count(0),
      recent_errors(false) {
  // empty
}

