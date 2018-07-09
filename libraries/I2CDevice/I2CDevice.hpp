//
// libraries/I2CDevice/I2CDevice.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/06/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#ifndef LIBRARIES_I2CDEVICE_I2CDEVICE_HPP
#define LIBRARIES_I2CDEVICE_I2CDEVICE_HPP

#include <i2c_t3.h>

/*! i2c error code bit masks */
#define I2C_ERR_MASK_TIMEOUT       0b00001;
#define I2C_ERR_MASK_TRANSMISSION  0b11110;
#define I2C_ERR_MASK_DATA_TOO_LONG 0b00010;
#define I2C_ERR_MASK_NACK_ON_ADDR  0b00100;
#define I2C_ERR_MASK_NACK_ON_DATA  0b01000;
#define I2C_ERR_MASK_OTHER         0b10000;

class I2CDevice {

public:

  /*! Get and set the i2c response timeout in milliseconds. A timeout value of
   *  zero means no timeout.
   */
  void i2c_set_timeout(unsigned long i2c_timeout);
  uint8_t i2c_get_timeout() const;

protected:

  /*! Constructor with default timeout value of 0 */
  I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  /*! Constructor with user specified timeout value */
  I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned long i2c_timeout);

  /*! This function returns the cummulative error with this i2c device sense
   *  the last call to this function. To extract specific errors, and the
   *  returned error code with the defined error masks. For more specifics on
   *  the error types see the class documentation.
   */
  unsigned int i2c_pop_errors();

  /*! Identical to i2c_pop_errors() except the error parameter isn't zeroed */
  unsigned int i2c_peek_errors() const;

  /*! Write the specified byte array to the i2c device. No data request is
   * made. Check i2c_peek_errors() or i2c_pop_errors() for potential errors.
   */
  void i2c_write_bytes(uint8_t const *data, uint8_t len);

  /*! Identical to the following call: i2c_write_bytes(&data, 1) */
  void i2c_write_byte(uint8_t const &data);

  /*! Reads the requested number of bytes over i2c into the specified byte
   *  array. Check i2c_peek_errors() or i2c_pop_errors() for potential errors.
   *  If an error did occur, the integrity of the data read is not guaranteed.
   */
  void i2c_read_bytes(uint8_t *data, uint8_t len);

  /*! Identical to the following call: i2c_read_bytes(&data, 1) */
  void i2c_read_byte(uint8_t &data);

private:

  /*! i2c bus this  */
  i2c_t3 &i2c_wire;

  /*! i2c address constant */
  uint8_t const i2c_addr;

  /*! i2c response timeout value in milliseconds */
  unsigned long i2c_timeout;

  /*! Cummulative i2c error */
  unsigned int i2c_error;

};

#endif
