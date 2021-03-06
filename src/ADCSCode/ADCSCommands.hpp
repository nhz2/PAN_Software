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

/** \addtogroup ADCS
 *  @{ **/

#ifndef PAN_ADCSCODE_ADCSCOMMANDS_HPP_
#define PAN_ADCSCODE_ADCSCOMMANDS_HPP_

/** \enum ADCSMode
 *  \brief Enumeration of all the ADCS modes that the ADCS system can be placed
 *         in.
 *
 *  The mode is changed by sending a different mode byte to the ADCS computer
 *  over serial communication. Sending the NO_CHANGE code will keep the ADCS
 *  system in the most recently specified mode. Note also that some modes
 *  require additional arguments to be considered valid. **/
enum ADCSMode : unsigned char {
  /** \brief Specifies the ADCS system should continue operating in the most
   *         recently specified mode.
   *
   *  If this mode byte is sent to the ADCS system, no change in mode will
   *  occur. **/
  NO_CHANGE = 0,
  /** \brief Sets the ADCS system in the large angle slew mode.
   *
   *  The ADCS system will attempt to hold the specified omega in an intertial
   *  frame. This can be used to point prograde, retrograde, etc. Additional
   *  arguments that must be specified for this mode include the desired
   *  angular rate in ECIF and an initial attitude quaternion. **/
  LARGE_ANGLE_SLEW = 1,
  /** \brief Sets the ADCS system in the hold attitude mode.
   *
   *  The ADCS system will attempt to hold the specified attitude in an
   *  inertial frame. This can be used to hold a power favorable attitude for
   *  periods of time (after a significant portion of a year the attitude will
   *  need to be updated to account for revolution around the sun). Additional
   *  arguments that must be specified for this mode are the attitude quaternion
   *  in ECIF. **/
  HOLD_ATTITUDE = 2,
  /** \brief Sets the ADCS system into a mode that commands no torques.
   *
   *  The ADCS system will continue to monitor attitude and determine it's
   *  orientation but will not act on anything. This will leave wheels at their
   *  current speed and turn MTRs off. No additional arguments are required. **/
  ZERO_TORQUE = 3,
  /** \brief Sets the ADCS system to bring the angular rate of the spacecraft to
   *         zero.
   *
   *  The ADCS system will attempt to bring its angular rate in an inertial
   *  frame to zero. The wheels will also be brought to within some tolerance of
   *  their zero angular momentum state. No additional arguments are required.
   * **/
  DETUMBLE = 4,
  /** \brief Sets the ADCS system to bring the angular velocity of the RWAs to
   *         zero.
   *
   *  The ADCS system will attempt to bring the angular rate of each of the
   *  reaction wheels to zero. RWA saturation warnings should be expected once
   *  the reaction wheel speed approaches zero. No additional arguments are
   *  required. Current attitude will be held. **/
  CONTROLLED_DESPIN = 5,
  /** \brief Sets the ADCS system to safehold mode.
   *
   *  The ADCS system will turn off all of it's attitude control peripherals.
   *  This means the RWAs and MTRs will be turned off and attitude will be
   *  uncontrolled. Attitude determination will continue. No additional
   *  arguments are required. **/
  SAFEHOLD = 6
};

/** \enum ADCSRequest
 *  \brief Enumeration of all the data requests that can be sent to the ADCS
 *         system.
 *
 *  Once a data request is sent to the ADCS system, it will be queued on the
 *  ADCS computer and responded to in the first comms frame in which space is
 *  available. Each request has a specific payload format that will be
 *  specified here. **/
enum ADCSRequest : unsigned char {
  /** \brief Returns general timestamped attitude status of the spacecraft.
   *
   *  Specifically, this includes the spacecraft's attitude in ECIF, uncertainty
   *  in attitude, angular rate in ECIF, uncertainty in anglular rate, and a
   *  timestamp for the provided readings. **/
  MISSION_DATA = 0,
  /** \brief Returns the current sun sensor assembly.
   *
   *  Specifically, this returns an is funcitonal bit followed by the most
   *  recent set of twelve bit voltage readings off of each ADC. Lastly, the
   *  most recent confidence in sun vector determination is returned. **/
  SENSOR_HEALTH_SSA = 1,
  /** \brief Returns the current state of the MTRs.
   *
   *  Specifically, this includes an is funcitonal bit, current signed PWM
   *  value, and the MTR axis magnetometer reading for each MTR. **/
  SENSOR_HEALTH_MTR = 2,
  /** \brief Returns the current state of the reaction wheel assembly.
   *
   *  Specifically, this includes an is functional bit, current speed, desired
   *  speed, current acceleration ramp, and desired ramp for each reaction
   *  wheel. **/
  SENSOR_HEALTH_RWA = 3,
  /** \brief Returns the current state of the main IMU.
   *
   *  Specifically, this includes the most recent gyroscope and magnetometer
   *  readings in the body frame of the spacecraft along with an is functional
   *  bit for both devices. **/
  SENSOR_HEALTH_IMU = 4
};

/** \enum ADCSWarning
 *  \brief Enumeration of all the warning codes the ADCS system can report.
 *
 *  The ADCS system has the ability to return hardware and software warnings
 *  over serial communication. The following enum specifies the numerical value
 *  of each warning. Note that each warning will be represented as a single
 *  unsigned char when being transmitted over serial. Any warning ending with
 *  _NOT_FUNCTIONAL is tied to a devices required is_functional function. **/
enum ADCSWarning : unsigned char {
  /** \brief Sun sensor ADC 1 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, it means that ADC 1 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC1_NOT_FUNCTIONAL = 0,
  /** \brief Sun sensor ADC 2 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, it means that ADC 2 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC2_NOT_FUNCTIONAL = 1,
  /** \brief Sun sensor ADC 3 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, it means that ADC 3 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC3_NOT_FUNCTIONAL = 2,
  /** \brief Sun sensor ADC 4 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, it means that ADC 4 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC4_NOT_FUNCTIONAL = 3,
  /** \brief Sun sensor ADC 5 is not responding to i2c communication.
   *
   *  If this error is reported by the ADCS system, it means that ADC 5 has
   *  been marked as not functional. The device will remain disabled unless
   *  action is taken by the flight computer. **/
  SSA_ADC5_NOT_FUNCTIONAL = 4,
  /** \brief Sun sensor array has high uncertainty in determining a sun vector.
   *
   *  If this error is reported by the ADCS system, it means that that sun
   *  sensor array most likely has poor visibility of the sun or the satellite
   *  is in eclipse. This error is of little concern unless it persists for an
   *  extended period of time (i.e. more than a full orbit or ~90 minutes). **/
  SSA_LOW_CONFIDANCE = 5,
  /** \brief MTR along the x axis of the ADCS system is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the MTR
   *  along the x axis in the body frame of the spacecraft is not producing
   *  a magnetic field of noticable size - i.e. the magnetometer paired with it
   *  is not detecting a significant field while the MTR is on. **/
  MTR_X_NOT_FUNCTIONAL = 6,
  /** \brief MTR along the y axis of the ADCS system is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the MTR
   *  along the y axis in the body frame of the spacecraft is not producing
   *  a magnetic field of noticable size - i.e. the magnetometer paired with it
   *  is not detecting a significant field while the MTR is on. **/
  MTR_Y_NOT_FUNCTIONAL = 7,
  /** \brief MTR along the z axis of the ADCS system is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the MTR
   *  along the z axis in the body frame of the spacecraft is not producing
   *  a magnetic field of noticable size - i.e. the magnetometer paired with it
   *  is not detecting a significant field while the MTR is on. **/
  MTR_Z_NOT_FUNCTIONAL = 8,
  /** \brief Magnetometer paired with the MTR along the x axis is not
   *         functional.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  magnetometer paired with the MTR along the x axis of the body frame is
   *  not responding to i2c communication. The device will remain disabled
   *  unless action is taken by the flight computer. **/
  MTR_MAG_X_NOT_FUNCTIONAL = 9,
  /** \brief Magnetometer paired with the MTR along the y axis is not
   *         functional.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  magnetometer paired with the MTR along the y axis of the body frame is
   *  not responding to i2c communication. The device will remain disabled
   *  unless action is taken by the flight computer. **/
  MTR_MAG_Y_NOT_FUNCTIONAL = 10,
  /** \brief Magnetometer paired with the MTR along the z axis is not
   *         functional.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  magnetometer paired with the MTR along the z axis of the body frame is
   *  not responding to i2c communication. The device will remain disabled
   *  unless action is taken by the flight computer. **/
  MTR_MAG_Z_NOT_FUNCTIONAL = 11,
  /** \brief Gryo paired with the MTR along the x axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the gyro
   *  paired with the MTR along the x axis of the body frame is not responding
   *  to i2c communication. The device will remain disabled unless action is
   *  taken by the flight computer. Note, this device will be power down under
   *  normal operation and is used infrequently. **/
  MTR_GYR_X_NOT_FUNCTIONAL = 12,
  /** \brief Gryo paired with the MTR along the y axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the gyro
   *  paired with the MTR along the y axis of the body frame is not responding
   *  to i2c communication. The device will remain disabled unless action is
   *  taken by the flight computer. Note, this device will be power down under
   *  normal operation and is used infrequently. **/
  MTR_GYR_Y_NOT_FUNCTIONAL = 13,
  /** \brief Gryo paired with the MTR along the z axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means that the gyro
   *  paired with the MTR along the z axis of the body frame is not responding
   *  to i2c communication. The device will remain disabled unless action is
   *  taken by the flight computer. Note, this device will be power down under
   *  normal operation and is used infrequently. **/
  MTR_GYR_Z_NOT_FUNCTIONAL = 14,
  /** \brief Reaction wheel along the x axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means the the reaction
   *  wheel along the x axis of the body frame is not responding properly to
   *  speed, acceleration, or direction commands and should be powered down as
   *  soon as possible by the flight computer. **/
  RWA_X_NOT_FUNCTIONAL = 15,
  /** \brief Reaction wheel along the y axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means the the reaction
   *  wheel along the y axis of the body frame is not responding properly to
   *  speed, acceleration, or direction commands and should be powered down as
   *  soon as possible by the flight computer. **/
  RWA_Y_NOT_FUNCTIONAL = 16,
  /** \brief Reaction wheel along the z axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means the the reaction
   *  wheel along the z axis of the body frame is not responding properly to
   *  speed, acceleration, or direction commands and should be powered down as
   *  soon as possible by the flight computer. **/
  RWA_Z_NOT_FUNCTIONAL = 17,
  /** \brief Reaction wheel along the x axis is halfway to saturation.
   *
   *  If this warning is reported by the ADCS system, it means that the
   *  reaction wheel along the x axis of the body frame's speed is approaching
   *  it's maximum value in either the positve or negative direction. No
   *  immediate action is required. **/
  RWA_X_HALF_SATURATED = 18,
  /** \brief Reaction wheel along the y axis is halfway to saturation.
   *
   *  If this warning is reported by the ADCS system, it means that the
   *  reaction wheel along the y axis of the body frame's speed is approaching
   *  it's maximum value in either the positve or negative direction. No
   *  immediate action is required. **/
  RWA_Y_HALF_SATURATED = 19,
  /** \brief Reaction wheel along the z axis is halfway to saturation.
   *
   *  If this warning is reported by the ADCS system, it means that the
   *  reaction wheel along the z axis of the body frame's speed is approaching
   *  it's maximum value in either the positve or negative direction. No
   *  immediate action is required. **/
  RWA_Z_HALF_SATURATED = 20,
  /** \brief Reaction wheel along the x axis is saturated.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  reaction wheel along the x axis of the body frame's speed is at it's
   *  maximum value and the ADCS is therefore not able to hold the desired
   *  attitude. Time must be given to allow the MTRs to desaturate the wheels.
   *  Action may be required depending on the mission phase (e.g. docking needs
   *  high pointing accuracy so this error should force the flight computer to
   *  to take action). **/
  RWA_X_SATURATED = 21,
  /** \brief Reaction wheel along the y axis is saturated.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  reaction wheel along the y axis of the body frame's speed is at it's
   *  maximum value and the ADCS is therefore not able to hold the desired
   *  attitude. Time must be given to allow the MTRs to desaturate the wheels.
   *  Action may be required depending on the mission phase (e.g. docking needs
   *  high pointing accuracy so this error should force the flight computer to
   *  to take action). **/
  RWA_Y_SATURATED = 22,
  /** \brief Reaction wheel along the z axis is saturated.
   *
   *  If this error is reported by the ADCS system, it means that the
   *  reaction wheel along the z axis of the body frame's speed is at it's
   *  maximum value and the ADCS is therefore not able to hold the desired
   *  attitude. Time must be given to allow the MTRs to desaturate the wheels.
   *  Action may be required depending on the mission phase (e.g. docking needs
   *  high pointing accuracy so this error should force the flight computer to
   *  to take action). **/
  RWA_Z_SATURATED = 23,
  /** \brief Analog ramp for the RWA along the x axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means one of two things.
   *  First, the potentiometer itself is returning false for is_functional in
   *  which case the potentiometer for all three RWAs would be throwing an
   *  error. Second, only the potentiometer paired with the motor along the x
   *  axis of the body frame is broken. In either case, the flight computer
   *  should take action to prevent a more catastrophic failure. **/
  RWA_POT_X_NOT_FUNCTIONAL = 24,
  /** \brief Analog ramp for the RWA along the y axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means one of two things.
   *  First, the potentiometer itself is returning false for is_functional in
   *  which case the potentiometer for all three RWAs would be throwing an
   *  error. Second, only the potentiometer paired with the motor along the y
   *  axis of the body frame is broken. In either case, the flight computer
   *  should take action to prevent a more catastrophic failure. **/
  RWA_POT_Y_NOT_FUNCTIONAL = 25,
  /** \brief Analog ramp for the RWA along the z axis is not functional.
   *
   *  If this error is reported by the ADCS system, it means one of two things.
   *  First, the potentiometer itself is returning false for is_functional in
   *  which case the potentiometer for all three RWAs would be throwing an
   *  error. Second, only the potentiometer paired with the motor along the z
   *  axis of the body frame is broken. In either case, the flight computer
   *  should take action to prevent a more catastrophic failure. **/
  RWA_POT_Z_NOT_FUNCTIONAL = 26,
  /** \brief Main magnetometer is not responding to communication.
   *
   *  If this error is reported by the ADCS system, it means that the main
   *  magnetometer used to take readings of earth's magnetic field is returning
   *  false for is_functional. The spacecraft can continue to run, but attitude
   *  uncertainty will become unbounded over time. Action should be taken by the
   *  flight computer. **/
  IMU_MAG_NOT_FUNCTIONAL = 27,
  /** \brief Main gyroscope is not responding to communication.
   *
   *  If this error is reported by the ADCS system, it means that the main
   *  gyroscope used to take readings of the spacecraft's angular rate is
   *  returning false for is_functional. The spacecraft can continue to run, but
   *  attitude uncertainty will become unbounded over time. Action should be
   *  taken by the flight computer. **/
  IMU_GYR_NOT_FUNCTIONAL = 28
};

#endif

/** @} **/