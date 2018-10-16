//
// Devices/Libraries.hpp
// PAN
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

/** \defgroup ADCS
 *  \brief Contains the flight code that will run directly on the ADCS computer.
 *
 *  This includes more complex ADCS specific drivers (e.g. the sun sensor lookup
 *  table algorithm), autocoded simulink/MATLAB control algorithms, sensor
 *  filtering, and a serial communication handler to interface with the ADCS
 *  computer. **/

/** \defgroup Libraries
 *  \brief Contains all device drivers and code that will be shared between
 *         the ADCS and flight computers.
 *
 *  The device driver's in this group should only perform the most basic
 *  functions like setting registers on the i2c chip and reading data from
 *  registers. No filtering should be incorporated. **/
