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

ADS1115::ADS1115(i2c_t3 &i2c_wire, uint8_t i2c_addr, unsigned int alert_pin) : I2CDevice(i2c_wire, i2c_addr, 0), alert_pin(alert_pin) {
  this->sample_delay = (1000 / (unsigned long) ADS1115_128_SPS) + 1;
  this->alert_config_needed = true;
  this->sample_rate = ADS1115_128_SPS;
  this->gain = ADS1115_GAIN_TWO;
}

void ADS1115::set_gain(uint16_t gain) {
  this->gain = gain;
}

uint16_t ADS1115::get_gain() const {
  return this->gain;
}

static unsigned long const sample_delays[] = {
  8, 16, 32, 64, 128, 250, 475, 860
};

void ADS1115::set_sample_rate(uint16_t sample_rate) {
  this->sample_delay = (1000 / sample_delays[sample_rate >> 5]) + 1;
  this->sample_rate = sample_rate;
}

uint16_t ADS1115::get_sample_rate() const {
  return this->sample_rate;
}

static uint8_t const high_thresh[] = {
  0x02, 0x7F, 0xFF
};

static uint8_t const low_thresh[] = {
  0x03, 0x80, 0x00
};

void ADS1115::start_read(unsigned int line) {
  // Check if alert pin config needed
  if(this->alert_config_needed) {
    i2c_write_bytes(high_thresh, 3);
    i2c_write_bytes(low_thresh, 3);
    this->alert_config_needed = false;
  }
  // Write the configuration signal
  uint16_t config = 0x8108 | this->sample_rate | this->gain | (((line % 4) << 12) + 0x4000);
  uint8_t arr[] = {
    0x01,
    (uint8_t)(config >> 0x8),
    (uint8_t)(config & 0xFF)
  };
  i2c_write_bytes(arr, 3);
  // Record conversion start time
  this->timestamp = millis();
}

bool ADS1115::end_read(int16_t &val) {
  // Wait for alert pin with timeout option
  unsigned long offset = 0;
  if(this->timestamp + (this->sample_delay << 7) < this->timestamp)
    offset = (this->sample_delay << 7);
  while(digitalRead(this->alert_pin) == LOW && millis() + offset <= this->timestamp + offset + this->sample_delay);
  // Request and read response
  uint8_t data[2];
  i2c_write_byte(0x00);
  i2c_read_bytes(data, 2);
  if(!i2c_pop_errors()) {
    val = (int16_t) ((data[0] << 8) | data[1]);
    return true;
  }
  // Failed read
  this->alert_config_needed = true;
  return false;
}

bool ADS1115::read(unsigned int line, int16_t &val) {
  this->start_read(line);
  return this->end_read(val);
}
