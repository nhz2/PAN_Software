//
// libraries/PololuIMU/PololuIMU.cpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "PololuIMU.hpp"

// LIS3MDL class implementation ----------------------------------------------------

LIS3MDL::LIS3MDL(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {
  for(int i = 0; i < 3; i++)
    this->mag[i] = 0;
  this->sample_frequency = LIS3MDL_SF_0_625;
  this->xy_performance = LIS3MDL_PERF_LOW;
  this->z_performance = LIS3MDL_PERF_LOW;
  this->scale = LIS3MDL_SCALE_PM4G;
  this->should_reset = true;
}

void LIS3MDL::set_sample_frequency(uint8_t sample_frequency) {
  this->sample_frequency = sample_frequency;
}

uint8_t LIS3MDL::get_sample_frequency() const {
  return this->sample_frequency;
}

void LIS3MDL::set_xy_performance(uint8_t xy_performance) {
  this->xy_performance = xy_performance;
}

uint8_t LIS3MDL::get_xy_performance() const {
  return this->xy_performance;
}

void LIS3MDL::set_z_performance(uint8_t z_performance) {
  this->z_performance = z_performance;
}

uint8_t LIS3MDL::get_z_performance() const {
  return this->z_performance;
}

void LIS3MDL::set_scale(uint8_t scale) {
  this->scale = scale;
}

uint8_t LIS3MDL::get_scale() const {
  return this->scale;
}

int16_t LIS3MDL::x() const {
  return this->mag[0];
}

int16_t LIS3MDL::y() const {
  return this->mag[1];
}

int16_t LIS3MDL::z() const {
  return this->mag[2];
}

bool LIS3MDL::read() {
  // Update configuration if needed
  if(this->should_reset) {
    uint8_t arr[] = {
      LIS3MDL_REG_CTRL1, (uint8_t) ((this->xy_performance << 5) | this->sample_frequency),
      LIS3MDL_REG_CTRL2, this->scale,
      LIS3MDL_REG_CTRL3, 0,
      LIS3MDL_REG_CTRL4, (uint8_t) (this->z_performance << 2),
      LIS3MDL_REG_CTRL5, 0
    };
    for(int i = 0; i < 10; i += 2)
      i2c_write_bytes(&arr[i], 2);
    // Process potential configuration errors
    if(i2c_pop_errors())
      return false;
  }
  // Set register and request data
  uint8_t data[8];
  i2c_write_byte(LIS3MDL_REG_OUT_XL | 0x80);
  i2c_read_bytes(data, 8);
  if(i2c_pop_errors())
    return false;
  // Successful read
  for(int i = 0; i < 3; i++)
    this->mag[i] = (int16_t) ((data[2 * i + 1] << 8) | data[2 * i]);
  this->should_reset = false;
  return true;
}

//
