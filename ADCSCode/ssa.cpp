//
// ADCSCode/ssa.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 7/24/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include <vector>

#include "ssa.hpp"

ADS1015 ssa::adcs[5] = {
  ADS1015(Wire,  ADS1015::ADDR::GND, 11),
  ADS1015(Wire,  ADS1015::ADDR::VDD, 12),
  ADS1015(Wire,  ADS1015::ADDR::SCL, 14),
  ADS1015(Wire1, ADS1015::ADDR::VDD, 13),
  ADS1015(Wire1, ADS1015::ADDR::SDA, 20)
};

int16_t ssa::raw_data[20] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

unsigned int ssa::consec_err[5] = {
  0, 0, 0, 0, 0
};

/*! Helper function to access raw_data entries */
inline int16_t &ref_raw_data(unsigned int adc, unsigned int line) {
  return ssa::raw_data[4 * adc + line];
}

/*! Specifies the next line of the ADCs that should be read by read */
static unsigned int next_line;

void ssa::init() {
  // Congigure the ADCs
  for(unsigned int i = 0; i < 5; i++) {
    adcs[i].set_sample_rate(ADS1015::SR::SPS_920);
    adcs[i].set_gain(ADS1015::GAIN::ONE);
  }
  // Initial reads from all adc channels
  for(unsigned int line = 0; line < 4; line++) {
    // Initiate first read
    for(unsigned int adc = 0; adc < 5; adc++)
      adcs[adc].start_read(line);
    // Attempt to complete first read
    for(unsigned int adc = 0; adc < 5; adc++) {
      if(!adcs[adc].end_read(ref_raw_data(adc, line))) {
        ref_raw_data(adc, line) = 0;
        consec_err[adc]++;
      }
    }
  }
  // Initialize next_line
  next_line = 0;
}

bool ssa::read(float *sun_vector) {
  // Generate a list of which ADCs to read
  std::vector<unsigned int> adc_vec;
  adc_vec.reserve(5);
  for(unsigned int i = 0; i < 5; i++)
    if(consec_err[i] < 100)
      adc_vec.push_back(i);
  // Iterate through the functioning ADCs
  for(unsigned int adc : adc_vec)
    adcs[adc].start_read(next_line);
  for(unsigned int adc : adc_vec) {
    if(!adcs[adc].end_read(ref_raw_data(adc, next_line))) {
      ref_raw_data(adc, next_line) = 0;
      consec_err[adc]++;
    }
  }
  // Increment the next line
  next_line = (next_line + 1) % 4;
  // Determine the sun_vector
  // TODO : Actually do this
  sun_vector[0] = 0.0f;
  sun_vector[1] = 0.0f;
  sun_vector[2] = 1.0f;
  return false;
}
