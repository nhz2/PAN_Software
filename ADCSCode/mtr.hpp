//
// ADCSCode/mtr.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 7/24/2018
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

/*! This enumerates the three ways an MTR can be asked to operate */
enum MTR_OPERATION {
  FORWARD,
  REVERSE,
  CONTROL
};

/*! Namespace for magnetic torque rod related things */
namespace mtr {

  

}

#endif
