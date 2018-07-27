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

bool I2CDevice::i2c_pop_errors() {
  bool temp = this->i2c_error;
  this->i2c_error = false;
  return temp;
}

bool I2CDevice::i2c_peek_errors() const {
  return this->i2c_error;
}

void I2CDevice::i2c_write_bytes(uint8_t const *data, uint8_t len) {
  // Write data over i2c
  this->i2c_wire.beginTransmission(this->i2c_addr);
  this->i2c_wire.write(data, len);
  // End transmission and process errors
  this->i2c_error |= this->i2c_wire.endTransmission(I2C_STOP, this->i2c_timeout);
}

void I2CDevice::i2c_read_bytes(uint8_t *data, uint8_t len) {
  // Request data
  this->i2c_error |= ~this->i2c_wire.requestFrom(this->i2c_addr, len, I2C_STOP, this->i2c_timeout);
  // Read data
  for(int i = 0; i < len; i++)
    data[i] = this->i2c_wire.read();
}
