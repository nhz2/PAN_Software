//
// ADCSCode/mtr.cpp
//
// Contributors:
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "mtr.hpp"

mtr::MTRUnit mtr::mtrs[3] = {
  { 5,  6, CONTROL, 0, LIS3MDL( Wire, LIS3MDL_ADDR_HIGH), 0},
  {35, 36, CONTROL, 0, LIS3MDL(Wire1,  LIS3MDL_ADDR_LOW), 0},
  {29, 30, CONTROL, 0, LIS3MDL(Wire1, LIS3MDL_ADDR_HIGH), 0}
};

void mtr::init() {
  for(unsigned int i = 0; i < 3; i++) {
    mtr::MTRUnit &m = mtr::mtrs[i];
    // Initialize H bridge pins
    pinMode(m.f_pin, OUTPUT);
    pinMode(m.r_pin, OUTPUT);
    // Set magnetometer settings
    m.mag.set_xy_performance(LIS3MDL_PERF_LOW);
    m.mag.set_z_performance(LIS3MDL_PERF_HIGH);
    m.mag.set_sample_frequency(LIS3MDL_SF_80);
    m.mag.set_scale(LIS3MDL_SCALE_PM4G);
    // Read in first data points
    m.consec_err = (m.mag.read() ? 0 : m.consec_err + 1);
  }
}

void mtr::actuate(MTR_OPERATION op_x, MTR_OPERATION op_y, MTR_OPERATION op_z) {
  // Condense the inputs into an array
  MTR_OPERATION const ops[3] = { op_x, op_y, op_z };
  // Actuate the MTR units
  for(unsigned int i = 0; i < 3; i++) {
    mtr::MTRUnit &m = mtr::mtrs[i];
    if(ops[i] == MTR_OPERATION::CONTROL) {
      // Hysteresis mode control requires an update
      m.consec_err = (m.mag.read() ? 0 : m.consec_err + 1);
      // TODO : Make this fight hysteresis not just turn off the MTRs
      analogWrite(m.f_pin, 0);
      analogWrite(m.r_pin, 0);
      m.op_mode = ops[i];
      m.last_pwm = 0;
    } else {
      // Other modes don't need top be updated if they were already engaged
      if(ops[i] != m.op_mode) {
        m.op_mode = ops[i];
        if(ops[i] == MTR_OPERATION::FORWARD) {
          // Run MTR at full power in the foward direction
          analogWrite(m.r_pin, 0);
          analogWrite(m.f_pin, 255);
          m.last_pwm = 255;
        } else {
          // Run MTR at full power in the negative direction
          analogWrite(m.f_pin, 0);
          analogWrite(m.r_pin, 255);
          m.last_pwm = -255;
        }
      }
    }
  }
}
