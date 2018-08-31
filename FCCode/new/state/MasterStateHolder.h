#ifndef MASTER_STATE_HOLDER_H_
#define MASTER_STATE_HOLDER_H_

#include <iostream>
#include "HardwareAvailabilityTable.h"
#include "EEPROMController.h"

namespace PAN {
class MasterStateHolder {
  private:
    State fsm_state; // Finite State Machine state.

    // EEPROM-based variables
    EEPROMController eeprom;
    long number_of_reboots; // Number of software reboots experienced by satellite in its
                            // history. This is the RAM copy of this number; the actual
                            // copy is stored in EEPROM.
    bool reboot_had_logged_cause; // Did the current boot result from a fault?
    uint8_t reboot_cause; // What was the fault experienced that necessitated
                          // a reboot?

    HardwareAvailabilityTable havt; // Hardware Availability Table
  public:
    enum State
    {
      // Common Cubesat Modes
      DORMANT_CRUISE,
      INITIALIZATION,
      NORMAL_OPS,
      ECLIPSE,
      SAFE_HOLD,
      // PAN-specific modes
      NEAR_FIELD_RENDEZVOUS,
      DOCKED,
      LARGE_SLEW_MANUEVERS
    };

    /** Constructor. **/
    MasterStateHolder();

    /** Returns the hardware availability table. **/
    HardwareAvailabilityTable const &get_havt();

    ///////////////////// State Getters and Setters ////////////////////////
    /** Overrides the current state. **/
    void set_state(State state);

    /** Retrieves the current state of the satellite. */
    State get_state();
    ////////////////////////////////////////////////////////////////////////
};

extern MasterStateHolder msh;
}