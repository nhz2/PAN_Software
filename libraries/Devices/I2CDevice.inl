//
// Devices/I2CDevice.inl
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_DEVICES_I2CDEVICE_INL_
#define PAN_DEVICES_I2CDEVICE_INL_

namespace Devices {

namespace I2CDEVICE_V1 {
bool I2CDevice::dev_setup() {
  for (unsigned int i = 0; i < I2CDEVICE_DISABLE_AT; i++)
    if (this->i2c_ping()) return true;
  return false;
}

bool I2CDevice::dev_is_functional() const {
  return (this->error_count < I2CDEVICE_DISABLE_AT);
}

void I2CDevice::dev_reset() {
  this->error_count = 0;
  this->recent_errors = false;
}

void I2CDevice::dev_disable() {
  this->error_count = I2CDEVICE_DISABLE_AT;
  this->recent_errors = true;
}

inline void I2CDevice::i2c_set_timeout(unsigned long i2c_timeout) {
  this->timeout = i2c_timeout * 1000;
}

inline unsigned long I2CDevice::i2c_get_timeout() const {
  return this->timeout / 1000;
}

inline bool I2CDevice::i2c_data_is_valid() const {
  return this->error_count == 0;
}

I2CDevice::I2CDevice(i2c_t3 &wire, uint8_t addr, unsigned long timeout)
    : wire(wire),
      addr(addr),
      timeout(timeout),
      error_count(0),
      recent_errors(false) {
  // empty
}

inline bool I2CDevice::i2c_pop_errors() {
  bool temp = this->recent_errors;
  this->recent_errors = false;
  return temp;
}

inline bool I2CDevice::i2c_peek_errors() const { return this->recent_errors; }

template <typename T>
void I2CDevice::i2c_transmit_data(T const *data, std::size_t len, i2c_stop s) {
  this->i2c_begin_transmission();
  this->i2c_write(data, len);
  this->i2c_end_transmission(s);
}

template <typename T>
void I2CDevice::i2c_receive_data(T *data, std::size_t len, i2c_stop s) {
  this->i2c_request_from(len * sizeof(T), s);
  if (this->i2c_peek_errors()) return;
  for (std::size_t i = 0; i < len * sizeof(T); i++)
    ((uint8_t *)data)[i] = this->i2c_read();
}

inline void I2CDevice::i2c_begin_transmission() {
  this->wire.beginTransmission(this->addr);
}

inline void I2CDevice::i2c_end_transmission(i2c_stop s) {
  bool err = (this->wire.endTransmission(s, this->timeout) != 0);
  this->recent_errors = (this->recent_errors || err);
}

inline void I2CDevice::i2c_send_transmission(i2c_stop s) {
  this->wire.sendTransmission(s);
}

inline void I2CDevice::i2c_request_from(std::size_t len, i2c_stop s) {
  bool err = (this->wire.requestFrom(this->addr, len, s, this->timeout) == 0);
  this->recent_errors = (this->recent_errors || err);
}

inline void I2CDevice::i2c_send_request(std:; size_t len, i2c_stop s) {
  this->wire.sendRequest(this->addr, len, s);
}

inline bool I2CDevice::i2c_done() const { return (this->wire.done() == 1) }

inline void I2CDevice::i2c_finish() {
  bool err = (this->wire.finish(this->timeout) == 0);
  this->recent_errors = (this->recent_errors || err);
}

inline void I2CDevice::i2c_write(uint8_t data) {
  bool err = (this->wire.write(data) == 0);
  this->recent_errors = (this->recent_errors || err);
}

template <typename T>
inline void I2CDevice::i2c_write(T const *data, std::size_t len) {
  bool err = (this->wire.write((uint8_t *)data, len * sizeof(T)) == 0);
  this->recent_errors = (this->recent_errors || err);
}

inline void I2CDevice::i2c_available() const { return this->wire.available(); }

// TODO : Double check that this function is implemented properly
inline uint8_t I2CDevice::i2c_read() {
  int val = this->wire.read();
  bool err = (val == -1);
  this->recent_errors = (this->recent_errors || err);
  return (uint8_t)(val && 0xFF);
}

template <typename T>
inline void I2CDevice::i2c_read(T *data, std::size_t len) {
  bool err =
      (this->wire.read((uint8_t *)data, len * sizeof(T)) != len * sizeof(T));
  this->recent_errors = (this->recent_errors || err);
}

// TODO : Double check that this function is implemented properly
inline uint8_t i2c_peek() {
  int val = this->wire.peek();
  bool err = (val == -1);
  this->recent_errors = (this->recent_errors || err);
  return (uint8_t)(val && 0xFF);
}
}  // namespace I2CDEVICE_V1
}  // namespace Devices

#endif