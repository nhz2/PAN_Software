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

    /*! potentiometer to control speed ramp and it's communication error count */
    AD5254 set_ramp;
    unsigned int consec_err;

  };

  /*! The three wheel assemblies */
  extern WheelUnit wheels[3];

  /*! Initiates the pin modes of all wheel related pins.*/
  void init();


#ifdef VERBOSE
  /*! Outputs the operation modes and last pwm values to a csv style data line.
   *  The format of the line is as follows:
   *    op1,pwm1,magz1,err1,op2,...,magz3,err3
   */
  void verbose_output() {
    for(unsigned int i = 0; i < 3; i++) {
      WheelUnit const &w = wheels[i];
      Serial.print(String(analogRead(w.read_speed_pin) + ","));
      Serial.print(String(analogRead(w.read_ramp_pin) + ","));
      Serial.print(w.consec_err);
      if(i < 2)
        Serial.print(',');
    }
  }

  /*! Outputs alert messages on components with a non-zero consec_err value */
  void verbose_error() {
    for(unsigned int i = 0; i < 3; i++)
      if(wheels[i].consec_err > 0)
        Serial.println("!Wheel pot " + String(i) + " had a communication error");
  }
#endif
}







#endif
