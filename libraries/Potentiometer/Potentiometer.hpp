//
// libraries/Potentiometer/Potentiometer.hpp
//
// Created by Nathan Zimmerberg (nhz2@cornell.edu) on 07/30/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#ifndef LIBRARIES_POTENTIOMETER_POTENTIOMETER_HPP
#define LIBRARIES_POTENTIOMETER_POTENTIOMETER_HPP

#include <I2CDevice.hpp>


/*! Possible slave addresses */
static const uint8_t AD5254_ADDR_0 = 0b0101100;
static const uint8_t AD5254_ADDR_1 = 0b0101101;
static const uint8_t AD5254_ADDR_2 = 0b0101110;
static const uint8_t AD5254_ADDR_3 = 0b0101111;

class AD5254 : public I2CDevice {

public:

  /*! Constructs a AD5254 on the specified wire and with the given
   *  address.
   default states:
      rdac values are 0
   */
  AD5254(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  /*!Sets and returns the RDAC settings*/
  void set_rdac(uint8_t rdac0, uint8_t rdac1, uint8_t rdac2);
  uint8_t get_rdac0();
  uint8_t get_rdac1();
  uint8_t get_rdac2();

  /*!if sent i2c message to read rdacs
  updates value of rdacs*/
  // bool read_block();

  /*!if sent i2c message to write rdacs*/
  bool write_block();

  /*!if added i2c message to write rdacs to i2c queue
  adds a slave address and 4 data bytes to i2c_t3's bus.
  uses i2c stop.*/
  bool write_noblock();

private:
  /*! RDAC settings for 0,1,and 2*/
  uint8_t rdac[3];

};

#endif
