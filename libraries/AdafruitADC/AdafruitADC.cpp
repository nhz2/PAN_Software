//
// libraries/AdafruitADC/AdafruitADC.cpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "AdafruitADC.hpp"

// ADS1115 class implementation ------------------------------------------------

ADS1015::ADS1015(i2c_t3 &i2c_wire, ADDR i2c_addr, unsigned int alert_pin) : I2CDevice(i2c_wire, i2c_addr, 0), alert_pin(alert_pin) {
  this->sample_delay = 1;
  this->alert_config_needed = true;
  this->sample_rate = SR::SPS_1600;
  this->gain = GAIN::TWO;
}

static unsigned long const sample_rates[] = {
  128, 250, 490, 920, 1600, 2400, 3300
};

void ADS1015::set_sample_rate(SR sample_rate) {
  this->sample_delay = (1000 / sample_rates[sample_rate >> 5]) + 1;
  this->sample_rate = sample_rate;
}

static uint8_t const thresh[] = {
  0x02, 0x7F, 0xFF, 0x03, 0x80, 0x00
};

void ADS1015::start_read(unsigned int line) {
  // Check if alert pin config needed
  if(this->alert_config_needed) {
    // Write high thresh register
    i2c_write_bytes(&thresh[0], 3, I2C_NOSTOP);
    // Write low thresh register
    i2c_begin_transmission();
    i2c_write(&thresh[3], 3);
    i2c_send_transmission(I2C_NOSTOP);
  }
  // Write the configuration signal
  uint16_t config = 0x8108 | this->sample_rate | this->gain | (((line % 4) << 12) + 0x4000);
  uint8_t arr[] = {
    0x01,
    (uint8_t)(config >> 0x8),
    (uint8_t)(config & 0xFF)
  };
  i2c_finish();
  i2c_write_bytes(arr, 3, I2C_STOP);
  // Record conversion start time
  this->timestamp = millis();
}

bool ADS1015::end_read(int16_t &val) {
  // Wait for alert pin with timeout option
  unsigned long offset = 0;
  if(this->timestamp + (this->sample_delay << 7) < this->timestamp)
    offset = (this->sample_delay << 7);
  while(digitalRead(this->alert_pin) == LOW && millis() + offset <= this->timestamp + offset + this->sample_delay);
  // Request and read response
  uint8_t data[2];
  i2c_write_byte(0x00, I2C_NOSTOP);
  i2c_read_bytes(data, 2, I2C_STOP);
  if(i2c_pop_errors()) {
    // Failed
    this->alert_config_needed = true;
    return false;
  }
  // Success
  val = (int16_t) ((data[0] << 8) | data[1]);
  val = val >> 4;
  this->alert_config_needed = false;
  return true;
}

bool ADS1015::read(unsigned int line, int16_t &val) {
  this->start_read(line);
  return this->end_read(val);
}
