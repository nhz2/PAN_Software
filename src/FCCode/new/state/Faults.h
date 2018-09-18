#ifndef FAULTS_H_
#define FAULTS_H_

namespace PAN {
namespace Faults {
// Faults that can have various system responses.
// TODO Add descriptions for each of these.
enum SoftFaults
{
    OPERATING_TEMPERATURE_OUT_OF_BOUNDS,
    PRESSURE_TOO_HIGH,
    TANK_1_PRESSURE_DROPPING,
    VALVE_BETWEEN_TANKS_NOT_OPENING,
    BACKUP_VALVE_BETWEEN_TANKS_NOT_OPENING,
    OUT_OF_FUEL
};

// Faults that necessitate the system going into a Safe Hold.
// TODO Add descriptions for each of these.
enum HardFaults
{
    // Propulsion
    SURVIVAL_TEMPERATURE_OUT_OF_BOUNDS,
    PRESSURE_CRITICALLY_HIGH, // TODO inconsistency on spreadsheet exists.
    // ADCS
    OVERSPIN,
    PERSISTENT_OSCILLATION,
    WHEEL_SPEED_TOO_HIGH,
    COMMAND_OUT_OF_WHEEL_SPEED_RANGE,
    COMMAND_OUT_OF_TORQUE_CAPABILITY_RANGE,
    ACCELERATION_TOO_HIGH,
    SUN_SENSORS_INCONSISTENT_WITH_SELF,
    SUN_SENSORS_INCONSISTENT_WITH_GYRO,
    SUN_SENSORS_INCONSISTENT_WITH_MAGNETOMETER,
    SUN_SENSORS_VOLTAGE_OUT_OF_BOUNDS,
    MAGNETOMETER_OUT_OF_BOUNDS,
    MAGNETOMETER_INCONSISTENT_WITH_GYRO,
    // Telemetry & Communications
    GPS_DATA_DOES_NOT_MATCH_ORBIT_PROPAGATOR,
    RELATIVE_VELOCITY_OUT_OF_BOUNDS,
    REPEATED_FALSE_LOCK,
    INCONSISTENT_RELATIVE_POSITION,
    INTERSAT_COMMS_FAILED_WITH_2KM_SEPARATION,
    // Power
    BATTERY_CHARGING_CURRENT_OUT_OF_BOUNDS,
    LOW_BATTERY_VOLTAGE,
    TEMPERATURE_OUT_OF_BOUNDS,
    CURRENT_DRAW_OUT_OF_BOUNDS
};

// Faults that necessitate the system going into critical power mode.
// TODO Add descriptions for each of these.
enum CriticalFaults {
    CRTICAL_BATTERY_VOLTAGE
};
}
}

#endif