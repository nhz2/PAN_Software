//
// ADCSCode/wheel.hpp
//
// Contributors:
//   Nathan Zimmerberg        nhz2@cornell.edu  09/2/2018
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/* This header file contains function declarations for the reaction wheels
 * incorporated into the ADCS system. The header is designed to be used with
 * the TESTING and VERBOSE macros as defined in ADCSCode/ADCSCode.ino. See
 * wheels.cpp for the function implementations.
 */
#ifndef ADCSCODE_WHEEL_HPP
#define ADCSCODE_WHEEL_HPP

#include <Potentiometer.hpp>

/*! Namespace for reaction wheel related things */
namespace wheel {

  /*! Struct containing all the data corresponding to a wheel and its
   *  peripherals.
   */
  struct WheelUnit {

    /*! digital and pwm pins */
    //Enable CCW: Digital Input 3,	High active
    unsigned int const en_ccw_pin;
    //Enable CW: Digital Input 2,	High active
    unsigned int const en_cw_pin;
    //PWM set speed: Digital Input 1, 0 to 5000 rpm : 10% to 90%
    unsigned int const set_speed_pin;

    /*! analog in out pins */
    //Speed read: Analog Output 2
    unsigned int const read_speed_pin;
    //Ramp read: reading of pot output
    unsigned int const read_ramp_pin;

    /*acceleration control states and parameters*/
    //speed_command = ks*accel_command+speedi
    //ramp_command = kr*accel_command
    //The last speed command sent to this wheel, units rad/s
    double speedi;
    //pwm out at 0.0 rad/s and pwm out slope
    //pwm out(speed)= (int)|round(pwm0 + pwm_slope*|speed|)|
    const double pwm0;
    const double pwm_slope;
    //pot out at 0.0 rad/s/s and pot out slope
    //pot out(ramp)= (int)|round(pot0 + pot_slope*|ramp|)|
    const double pot0;
    const double pot_slope;

  };
  /*! potentiometer to control speed ramp and it's communication error count */
  extern AD5254 set_ramp;
  extern unsigned int consec_err;

  //speed_command = ks*accel_command+speedi
  //ramp_command = kr*accel_command
  //speed constant, units s
  extern double ks;
  //ramp constant, units none
  extern double kr;

  /*! The three wheel assemblies */
  extern WheelUnit wheels[3];

  /*! Initiates the pin modes of all wheel related pins.*/
  void init();

  /*! command wheel accelerations units rad/s/s
    adds a slave address and 4 data bytes to the pot's i2c bus.*/
  void command_accel( double * accel);


#ifdef VERBOSE
  /*! Outputs the operation modes and last pwm values to a csv style data line.
   *  The format of the line is as follows:
   *    speed read,ramp read, pot error count, for x, y, and z
   */
  void verbose_output() {
    for(unsigned int i = 0; i < 3; i++) {
      WheelUnit const &w = wheels[i];
      Serial.print(String(analogRead(w.read_speed_pin) + ","));
      Serial.print(String(analogRead(w.read_ramp_pin) + ","));
      if(i < 2)
        Serial.print(',');
    }
  }

  /*! Outputs alert messages on components with a non-zero consec_err value */
  void verbose_error() {
    for(unsigned int i = 0; i < 3; i++)
      if(consec_err > 0)
        Serial.println("!Wheel pot " + String(i) + " had a communication error");
  }
#endif
}







#endif
