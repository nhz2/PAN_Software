//
// ADCSCode/wheels.hpp
//
// Contributors:
//   Nathan Zimmerberg        nhz2@cornell.edu  08/3/2018
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
#ifndef ADCSCODE_WHEELS_HPP
#define ADCSCODE_WHEELS_HPP

#include <Potentiometer.hpp>

/*! Namespace for reaction wheel related things */
namespace wheels {

  /*! Struct containing all the data corresponding to an wheel and its
   *  peripherals.
   */
  struct WheelUnit {

    /*! Forward and reverse PWM H bridge pins */
    unsigned int const f_pin;
    unsigned int const r_pin;

    /*! Current operation mode and last commanded signed PWM value */
    MTR_OPERATION op_mode;
    int last_pwm;

    /*! Monitoring magnetometer and it's communication error count */
    LIS3MDL mag;
    unsigned int consec_err;

  };

  /*! The three MTR assemblies */
  extern MTRUnit mtrs[3];

  /*! Initiates the pin modes of all MTR related pins. Each mtr is also set to
   *  a default MTR_OPERATION value of control.
   */
  void init();

  /*! Actuates the MTRs according to their operation mode. This function must
   *  be called frequently for the CONTROL operation to be effective.
   */
  void actuate(MTR_OPERATION op_x, MTR_OPERATION op_y, MTR_OPERATION op_z);

#ifdef VERBOSE
  /*! Outputs the operation modes and last pwm values to a csv style data line.
   *  The format of the line is as follows:
   *    op1,pwm1,magz1,err1,op2,...,magz3,err3
   */
  void verbose_output() {
    for(unsigned int i = 0; i < 3; i++) {
      MTRUnit const &m = mtrs[i];
      Serial.print(String(m.op_mode) + ",");
      Serial.print(String(m.last_pwm) + ",");
      Serial.print(String(m.mag.z()) + ",");
      Serial.print(String(m.op_mode) + ",");
      Serial.print(m.consec_err);
      if(i < 2)
        Serial.print(',');
    }
  }

  /*! Outputs alert messages on components with a non-zero consec_err value */
  void verbose_error() {
    for(unsigned int i = 0; i < 3; i++)
      if(mtrs[i].consec_err > 0)
        Serial.println("!MTR mag " + String(i) + " had a communication error");
  }
#endif








#endif
