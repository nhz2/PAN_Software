//
// ADCSCode/wheel.hpp
//
// Contributors:
//   Nathan Zimmerberg  nhz2@cornell.edu
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "wheel.hpp"

wheel::WheelUnit wheel::wheels[3] = {
  {39,26,23,A14,A1,AD5254(Wire1,AD5254_ADDR_1),0},
  {39,26,23,A14,A1,AD5254(Wire1,AD5254_ADDR_1),0},
  {39,26,23,A14,A1,AD5254(Wire1,AD5254_ADDR_1),0}
};

void wheel::init() {
  for(int i = 0; i < 3; i++) {
    pinMode(wheel::wheels[i].en_ccw_pin,OUTPUT);
    pinMode(wheel::wheels[i].en_cw_pin,OUTPUT);
    pinMode(wheel::wheels[i].set_speed_pin,OUTPUT);
    pinMode(wheel::wheels[i].read_speed_pin, INPUT);
    pinMode(wheel::wheels[i].read_ramp_pin, INPUT);
  }
}
