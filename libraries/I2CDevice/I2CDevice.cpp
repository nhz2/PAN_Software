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

void I2CDevice::i2c_set_timeout(unsigned long i2c_timeout) {
  this->i2c_timeout = i2c_timeout;
}

uint8_t I2CDevice::i2c_get_timeout() const {
  return this->i2c_timeout;
}

I2CDevice::I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr) : i2c_wire(i2c_wire), i2c_addr(i2c_addr) {
  this->i2c_timeout = 0;
  this->i2c_error = 0;
}

I2CDevice::I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned long i2c_timeout) : i2c_wire(i2c_wire), i2c_addr(i2c_addr) {
  this->i2c_timeout = i2c_timeout;
  this->i2c_error = 0;
}

unsigned int I2CDevice::i2c_pop_errors() {
  unsigned int temp = this->i2c_error;
  this->i2c_error = 0;
  return temp;
}

unsigned int I2CDevice::i2c_peek_errors() const {
  return this->i2c_error;
}

void I2CDevice::i2c_write_bytes(uint8_t const *data, uint8_t len) {
  // Write data over i2c
  this->i2c_wire.beginTransmission(this->i2c_addr);
  this->i2c_wire.write(data, len);
  // End transmission and process errors
  unsigned int i2c_error = this->i2c_wire.endTransmission();
  this->i2c_error |= (i2c_error ? 1 << i2c_error : 0);
}

void I2CDevice::i2c_write_byte(uint8_t const &data) {
  this->i2c_write_bytes(&data, 1);
}

void I2CDevice::i2c_read_bytes(uint8_t *data, uint8_t len) {
  // Request data
  this->i2c_wire.requestFrom(this->i2c_addr, len);
  // Look for timer overflow and prevent related errors
  unsigned long offset = 0;
  unsigned long stamp = millis();
  if(stamp + (this->i2c_timeout << 3) < stamp)
    offset = (this->i2c_timeout << 3);
  // Check for data arrival with timeout protection
  while(this->i2c_wire.available() < len) {
    if(this->i2c_timeout && millis() + offset > stamp + offset + this->i2c_timeout) {
      // Timeout has occurred
      this->i2c_error |= 1;
      return;
    }
  }
  // Read data
  for(int i = 0; i < len; i++)
    data[i] = this->i2c_wire.read();
}

void I2CDevice::i2c_read_byte(uint8_t &data) {
  this->i2c_read_bytes(&data, 1);
}
