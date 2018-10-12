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
// see http://mb-raw.blogspot.com/2018/02/teensy-and-pwm.html for pwm frequency

#include "wheel.hpp"
AD5254 wheel::set_ramp(Wire1,AD5254_ADDR_1);
unsigned int consec_err= 0;
double wheel::ks= 0.001;
double wheel::kr= 1.0;
wheel::WheelUnit wheel::wheels[3] = {
  {39,26,23,A14,A1,0.0,600.0,4.8,0.0,0.823},
  {39,26,23,A14,A1,0.0,600.0,4.8,0.0,0.823},
  {39,26,23,A14,A1,0.0,600.0,4.8,0.0,0.823}
};

void wheel::init() {
  for(int i = 0; i < 3; i++) {
    pinMode(wheel::wheels[i].en_ccw_pin,OUTPUT);
    pinMode(wheel::wheels[i].en_cw_pin,OUTPUT);
    pinMode(wheel::wheels[i].set_speed_pin,OUTPUT);
    pinMode(wheel::wheels[i].read_speed_pin, INPUT);
    pinMode(wheel::wheels[i].read_ramp_pin, INPUT);
    //http://mb-raw.blogspot.com/2018/02/teensy-and-pwm.html
    analogWriteFrequency(wheel::wheels[i].set_speed_pin, 10000);
  }
}

void wheel::command_accel( double * accel){
  //ramp change
  set_ramp.set_rdac((uint8_t)(abs(wheels[0].pot0+wheels[0].pot_slope*kr*abs(accel[0]))),
                    (uint8_t)(abs(wheels[1].pot0+wheels[1].pot_slope*kr*abs(accel[1]))),
                    (uint8_t)(abs(wheels[2].pot0+wheels[2].pot_slope*kr*abs(accel[2]))));
  set_ramp.write_noblock();

  //speed change
  double speedf;
  int speedf_pwm;
  for (int i= 0; i<3; i++){
    speedf= ks*accel[i]+wheels[i].speedi;
    speedf_pwm=(int)(abs(wheels[i].pwm0+wheels[i].pwm_slope*abs(speedf)));
    if (wheels[i].speedi*speedf<0.0){
      //direction change
      if (speedf>0.0) {
        //change to CW rotation
        digitalWrite(wheels[i].en_ccw_pin,LOW);
        digitalWrite(wheels[i].en_cw_pin,HIGH);
      } else {
        //change to CCW rotation
        digitalWrite(wheels[i].en_cw_pin,LOW);
        digitalWrite(wheels[i].en_ccw_pin,HIGH);
      }
    }
    analogWrite(wheels[i].set_speed_pin,speedf_pwm);
  }

}
