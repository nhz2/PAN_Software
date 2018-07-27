//
// libraries/I2CDevice/I2CDevice.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/06/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/*
 */

#ifndef LIBRARIES_I2CDEVICE_I2CDEVICE_HPP
#define LIBRARIES_I2CDEVICE_I2CDEVICE_HPP

#include <i2c_t3.h>

class I2CDevice {

public:

   /*! Set this device's I2C timeout in milliseconds */
  inline void i2c_set_timeout(unsigned long i2c_timeout) {
    this->i2c_timeout = i2c_timeout;
  }

  /*! Returns this devices I2C timeout in milliseconds */
  inline uint8_t i2c_get_timeout() const {
    return this->i2c_timeout;
  }

protected:

  /*! Constructor with default timeout value of 0 */
  I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  /*! Constructor with user specified timeout value */
  I2CDevice(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned long i2c_timeout);

  /*! Returns whether or not the device has experienced an error sense the last
   *  call to pop_errors. The error history parameter is then set back to false.
   */
  inline bool i2c_pop_errors() {
    bool temp = this->i2c_error;
    this->i2c_error = false;
    return temp;
  }

  /*! Peeks the error history parameter */
  inline bool i2c_peek_errors() const {
    return this->i2c_error;
  }

  /*! Begins an i2c transmission on the current bus */
  inline void i2c_begin_transmission() {
    this->i2c_wire.beginTransmission(this->i2c_addr);
  }

  /*! Ends the transmission on this bus */
  inline void i2c_end_transmission(i2c_stop s = I2C_STOP) {
    this->i2c_error |= this->i2c_wire.endTransmission(s, this->i2c_timeout);
  }

  /*! Sends the transmission on this bus (non-blocking version of above) */
  inline void i2c_send_transmission(i2c_stop s = I2C_STOP) {
    this->i2c_wire.sendTransmission(s);
  }

  /*! Sets the callback for a completed transmission. Note this affects the
   *  entire bus.
   */
  inline void i2c_on_transmission_done(void (*f)()) {
    this->i2c_wire.onTransmitDone(f);
  }

  /*! Request len bytes of data on this bus */
  inline void i2c_request_from(uint8_t len, i2c_stop s = I2C_STOP) {
    this->i2c_error |= this->i2c_wire.requestFrom(this->i2c_addr, len, s, this->i2c_timeout);
  }

  /*! Sends a data request on this bus (non-blocking version of above) */
  inline void i2c_send_request(uint8_t len, i2c_stop s = I2C_STOP) {
    this->i2c_wire.sendRequest(this->i2c_addr, len, s);
  }

  /*! Sets the callback for a completed request. Note this affects the entire
   *  bus.
   */
  inline void i2c_on_request_done(void (*f)()) {
    this->i2c_wire.onReqFromDone(f);
  }

  /*! Writes a byte to the outgoing buffer */
  inline void i2c_write(uint8_t data) {
    this->i2c_wire.write(data);
  }

  /*! Writes bytes to the outgoing buffer */
  inline void i2c_write(uint8_t const *data, unsigned int len) {
    this->i2c_wire.write(data, len);
  }

  /*! Waits for the completion of a non-blocking task on this bus */
  inline void i2c_finish() {
    this->i2c_error |= this->i2c_wire.finish(this->i2c_timeout);
  }

  /*! Write the specified byte array to the i2c device. No data request is
   *  made. Check i2c_peek_errors() or i2c_pop_errors() for potential errors.
   */
  void i2c_write_bytes(uint8_t const *data, unsigned int len);

  /*! Writes a single byte over i2c */
  inline void i2c_write_byte(uint8_t const &data) {
    this->i2c_write_bytes(&data, 1);
  }

  /*! Reads the requested number of bytes over i2c into the specified byte
   *  array. Check i2c_peek_errors() or i2c_pop_errors() for potential errors.
   *  If an error did occur, the integrity of the data read is not guaranteed.
   */
  void i2c_read_bytes(uint8_t *data, unsigned int len);

  /*! Reads a single byte over i2c */
  inline void i2c_read_byte(uint8_t &data) {
    this->i2c_read_bytes(&data, 1);
  }

private:

  /*! i2c bus this  */
  i2c_t3 &i2c_wire;

  /*! i2c address constant */
  uint8_t const i2c_addr;

  /*! i2c response timeout value in milliseconds */
  unsigned long i2c_timeout;

  /*! Cummulative i2c error */
  bool i2c_error;

};

#endif
