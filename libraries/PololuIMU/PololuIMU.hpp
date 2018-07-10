//
// libraries/PololuIMU/PololuIMU.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#ifndef LIBRARIES_POLOLUIMU_POLOLUIMU_HPP
#define LIBRARIES_POLOLUIMU_POLOLUIMU_HPP

#include <I2CDevice.hpp>

/*! Possible slave addresses */
static const uint8_t LIS3MDL_ADDR_HIGH = 0b0011110;
static const uint8_t LIS3MDL_ADDR_LOW  = 0b0011100;

/*! Magnetometer registers */
static const uint8_t LIS3MDL_REG_CTRL1  = 0x20;
static const uint8_t LIS3MDL_REG_CTRL2  = 0x21;
static const uint8_t LIS3MDL_REG_CTRL3  = 0x22;
static const uint8_t LIS3MDL_REG_CTRL4  = 0x23;
static const uint8_t LIS3MDL_REG_CTRL5  = 0x24;
static const uint8_t LIS3MDL_REG_OUT_XL = 0x28;
static const uint8_t LIS3MDL_REG_OUT_XH = 0x29;
static const uint8_t LIS3MDL_REG_OUT_YL = 0x2A;
static const uint8_t LIS3MDL_REG_OUT_YH = 0x2B;
static const uint8_t LIS3MDL_REG_OUT_ZL = 0x2C;
static const uint8_t LIS3MDL_REG_OUT_ZH = 0x2D;

/*! z-axis and xy-axis performance constants */
static const uint8_t LIS3MDL_PERF_LOW    = 0b0000;
static const uint8_t LIS3MDL_PERF_MEDIUM = 0b0100;
static const uint8_t LIS3MDL_PERF_HIGH   = 0b1000;
static const uint8_t LIS3MDL_PERF_ULTRA  = 0b1100;

/*! Possible sample frequencies */
static const uint8_t LIS3MDL_SF_0_625 = 0b00000;
static const uint8_t LIS3MDL_SF_1_25  = 0b00100;
static const uint8_t LIS3MDL_SF_2_5   = 0b01000;
static const uint8_t LIS3MDL_SF_5     = 0b01100;
static const uint8_t LIS3MDL_SF_10    = 0b10000;
static const uint8_t LIS3MDL_SF_20    = 0b10100;
static const uint8_t LIS3MDL_SF_40    = 0b11000;
static const uint8_t LIS3MDL_SF_80    = 0b11100;

/*! Possible magnetometer scaling values */
static const uint8_t LIS3MDL_SCALE_PM4G  = 0b0000000;
static const uint8_t LIS3MDL_SCALE_PM8G  = 0b0100000;
static const uint8_t LIS3MDL_SCALE_PM12G = 0b1000000;
static const uint8_t LIS3MDL_SCALE_PM16G = 0b1100000;

class LIS3MDL : public I2CDevice {

public:

  /*! Constructs a magnetometer on the specified wire and with the given
   *  address.
   */
  LIS3MDL(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  /*! Sets and returns the current sample frequency. The default sample
   * frequency is 0.625 Hz.
   */
  void set_sample_frequency(uint8_t sample_frequency);
  uint8_t get_sample_frequency() const;

  /*! Sets and returns the current xy component operating mode. The default mode
   * is low performace.
   */
  void set_xy_performance(uint8_t xy_performance);
  uint8_t get_xy_performance() const;

  /*! Sets and returns the current z component operating mode. The default mode
   * is low performace.
   */
  void set_z_performance(uint8_t z_performance);
  uint8_t get_z_performance() const;

  /*! Sets and returns the magnetometer scale value. The default full scale is
   * plus or minus 4 gauss.
   */
  void set_scale(uint8_t scale);
  uint8_t get_scale() const;

  /*! Returns magnetometer readings */
  int16_t x() const;
  int16_t y() const;
  int16_t z() const;

  /*! Reads the magnetometer and returns true if succesfful */
  bool read();

private:

  /*! Magnetic field readings */
  int16_t mag[3];

  /*! Configuration settings */
  uint8_t sample_frequency;
  uint8_t xy_performance;
  uint8_t z_performance;
  uint8_t scale;

  /*! Indicates whether control registers must be updated */
  bool should_reset;

};

/*!
 */
class LSM6DS33 {

public:

  LSM6DS33(i2c_t3 &i2c_wire, uint8_t i2c_addr);

private:

};

#endif
