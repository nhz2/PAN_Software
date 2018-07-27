//
// libraries/Util/Util.cpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/06/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "I2CDevice.hpp"

// I2CDevice class implementation ----------------------------------------------

I2CDevice::I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr) : i2c_wire(i2c_wire), i2c_addr(i2c_addr) {
  this->i2c_timeout = 0;
  this->i2c_error = 0;
}

I2CDevice::I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned long i2c_timeout) : i2c_wire(i2c_wire), i2c_addr(i2c_addr) {
  this->i2c_timeout = i2c_timeout;
  this->i2c_error = 0;
}

void I2CDevice::i2c_write_bytes(uint8_t const *data, unsigned int len) {
  this->i2c_wire.beginTransmission(this->i2c_addr);
  this->i2c_begin_transmission();
  this->i2c_write(data, len);
  this->i2c_end_transmission();
}

void I2CDevice::i2c_read_bytes(uint8_t *data, unsigned int len) {
  // Request data
  this->i2c_request_from(len);
  // Read data
  for(unsigned int i = 0; i < len; i++)
    data[i] = this->i2c_read();
}
