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
void ADS1015::single_comp_test() {
  Serial.print(String(this->read(0)) + "," + String(this->read(1)) + "," +
               String(this->read(2)) + "," + String(this->read(3)));
}

bool ADS1015::i2c_ping() {
  // TODO
  return true;
}

/** \brief Sample rate delays in milliseconds. **/
static unsigned long const sample_rates[] = {128,  250,  490, 920,
                                             1600, 2400, 3300};

ADS1015::ADS1015(i2c_t3 &wire, ADDR addr, unsigned long i2c_timeout,
                 unsigned int alert_pin)
    : I2CDevice(wire, addr, i2c_timeout), alert_pin(alert_pin) {
  this->timestamp = 0ul;
  this->alert_config_needed = true;
  this->set_sample_rate(SR::SPS_1600);
  this->set_gain(GAIN::PM_2_048V);
}

void ADS1015::set_sample_rate(SR sample_rate) {
  this->sample_delay = (1000 / sample_rates[sample_rate >> 5]) + 1;
  this->sample_rate = sample_rate;
}

void ADS1015::start_read(unsigned int channel) {
  // Check if the device needs to be configured
  if (this->alert_config_needed) {
    // Threshold configurationd data to enable alert pin
    static uint8_t const thresh[] = {0x02, 0x7F, 0xFF, 0x03, 0x80, 0x00};
    // Write high thresh register
    i2c_begin_transmission();
    i2c_write(&thresh[0], 3);
    i2c_end_transmission(I2C_NOSTOP);
    // Write low thresh register
    i2c_begin_transmission();
    i2c_write(&thresh[3], 3);
    i2c_send_transmission(I2C_NOSTOP);
  }
  // Determine the configuration signal
  uint16_t config = 0x8108 | this->sample_rate | this->gain |
                    (((channel % 4) << 12) + 0x4000);
  uint8_t arr[] = {0x01, (uint8_t)(config >> 0x8), (uint8_t)(config & 0xFF)};
  // Wait for non-blocking transmission and write configuration
  i2c_finish();
  i2c_begin_transmission();
  i2c_write(arr, 3);
  i2c_end_transmission();
  // Record conversion start time
  this->timestamp = millis();
}

int16_t ADS1015::end_read() {
  // Wait for alert pin with timeout option
  unsigned long offset = 0;
  if (this->timestamp + (this->sample_delay << 7) < this->timestamp)
    offset = (this->sample_delay << 7);
  while (digitalRead(this->alert_pin) == LOW &&
         millis() + offset <= this->timestamp + offset + this->sample_delay)
    ;
  // Configure register for data read
  i2c_begin_transmission();
  i2c_write(0x00);
  i2c_end_transmission(I2C_NOSTOP);
  // Request and read in data
  uint8_t data[2];
  i2c_request_from(2);
  i2c_read(data, 2);
  // Check for errors
  if (i2c_pop_errors()) {
    // Failed
    this->alert_config_needed = true;
    return false;
  }
  // Success
  int16_t x = (int16_t)((data[0] << 4) | (data[1] >> 4));
  this->alert_config_needed = false;
  return x;
}

int16_t ADS1015::read(unsigned int channel) {
  this->start_read(channel);
  return this->end_read();
}
}  // namespace ADS1015_V1
