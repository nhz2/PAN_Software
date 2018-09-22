//
// ADCSCode/ADCSWarnings.hpp
//
// Contributors:
//   Kyle Krol          kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

/** \addgroup ADCS
 *  @{ **/

#ifndef PAN_ADCSCODE_ADCSWARNINGS_HPP_
#define PAN_ADCSCODE_ADCSWARNINGS_HPP_

/** \brief Enumeration of all the warning codes the ADCS system can report.
 *
 *  The ADCS system has the ability to return hardware and software warnings
 *  over serial communication. The following enum specifies the numerical value
 *  of each warning. Note that each warning will be represented as a single
 *  unsigned char when being transmitted over serial. **/
enum ADCSWarnings : unsigned char {
  /** \brief Sun sensor ADC 1 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, this means that ADC 1 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC1_NOT_FUNCTIONAL = 0;
  /** \brief Sun sensor ADC 2 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, this means that ADC 2 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC2_NOT_FUNCTIONAL = 1;
  /** \brief Sun sensor ADC 3 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, this means that ADC 3 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC3_NOT_FUNCTIONAL = 2;
  /** \brief Sun sensor ADC 4 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, this means that ADC 4 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC4_NOT_FUNCTIONAL = 3;
  /** \brief Sun sensor ADC 5 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, this means that ADC 5 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC5_NOT_FUNCTIONAL = 4;

  /** \brief MTR along the x axis of the ADCS system is not functional.
   *
   *   **/
  MTR_X_NOT_FUNCTIONAL = 5;

};

#endif

/** @} **/