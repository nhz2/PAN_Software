//
// ADC1015.cpp
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "ADS1015.hpp"

namespace ADS1015_V1 {
String ADS1015::dev_sc_test() {
  return String(this->read(0)) + "," + String(this->read(1)) + "," +
         String(this->read(2)) + "," + String(this->read(3));
}

bool ADS1015::i2c_ping() {
  // TODO
  return false;
}

ADS1015::ADS1015(i2c_t3 &wire, ADS1015_ADDR addr, unsigned int alert_pin,
                 unsigned long timeout)
    : I2CDevice(wire, addr, timeout), alert_pin(alert_pin) {
  // TODO
}

void ADS1015::set_sample_rate(ADS1015_SR sample_rate) {
  // TODO
}

void ADS1015::start_read(unsigned int channel) {
  // TODO
}

int16_t ADS1015::end_read() {
  // TODO
  return 0;
}

int16_t ADS1015::read(unsigned int channel) {
  this->start_read(channel);
  return this->end_read();
}
}  // namespace ADS1015_V1
