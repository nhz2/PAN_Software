//
// ADCSCode/mtr.hpp
//
// Contributors:
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/* This header file contains function declarations for the magnetic torque rods
 * incorporated into the ADCS system. The header is designed to be used with
 * the TESTING and VERBOSE macros as defined in ADCSCode/ADCSCode.ino. See
 * mtr.cpp for the function implementations.
 */

#ifndef ADCSCODE_MTR_HPP
#define ADCSCODE_MTR_HPP
/*
#include <PololuIMU.hpp>

/*! How many times a sensor must fail before ignored
#define MTR_IGNORE_ON_READS 100

/*! This enumerates the three ways an MTR can be asked to operate
enum MTR_OPERATION {
  FORWARD,
  REVERSE,
  CONTROL
};

/*! Namespace for magnetic torque rod related things
namespace mtr {

  /*! MTR operation modes in the x, y, and z directions
  extern MTR_OPERATION op_modes[3];

  /*! Most recent PWM value written to each of the magnetic torque rods h
   *  bridges. The data is stored in the array as follows:
   *    f_mtr1, r_mtr1, f_mtr2,...,r_mtr3

  extern unsigned char last_pwm[6];

  /*! MTR monitoring magnetometer array
  extern LIS3MDL mags[3];

  /*! MTR monitoring magnetometer consecutive error count
  extern unsigned int consec_err[3];

  /*! Initiates the pin modes of all MTR related pins. Each mtr is also set to
   *  a default MTR_OPERATION value of control.

  void init();

  /*! Sets the mtr operation mode in the x direction
  inline void set_op_x(MTR_OPERATION op_mode) {
    op_modes[0] = op_mode;
  }

  /*! Sets the mtr operation mode in the y direction
  inline void set_op_y(MTR_OPERATION op_mode) {
    op_modes[1] = op_mode;
  }

  /*! Sets the mtr operation mode in the z direction
  inline void set_op_z(MTR_OPERATION op_mode) {
    op_modes[2] = op_mode;
  }

  /*! Actuates the MTRs according to their operation mode. This function must
   *  be called frequently for the CONTROL operation to be effective.

  void actuate();

#ifdef VERBOSE
  /*! Outputs the operation modes and last pwm values to a csv style data line.
   *  The format of the line is as follows:
   *    op1,op2,op3,f_mtr1,r_mtr1,f_mtr2,...,r_mtr3,mag1_z,...,err1,...,err3

  void verbose_output() {
    for(unsigned int i = 0; i < 3; i++)
      Serial.print(String(op_modes[i]) + ",");
    for(unsigned int i = 0; i < 6; i++)
      Serial.print(String(last_pwm[i]) + ",");
    for(unsigned int i = 0; i < 3; i++)
      Serial.print(String(mags[0].z()) + ",");
    Serial.print(String(consec_err[0]) + ",");
    Serial.print(String(consec_err[1]) + ",");
    Serial.print(String(consec_err[2]));
  }

  /*! Outputs alert messages on components with a non-zero consec_err value
  void verbose_error() {
    for(unsigned int i = 0; i < 3; i++)
      if(consec_err[i] > 0)
        Serial.println("!MTR mag " + String(i) + " had a communication error");
  }
#endif

}
*/
#endif
