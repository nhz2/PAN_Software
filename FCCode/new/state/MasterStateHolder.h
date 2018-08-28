#ifndef MASTER_STATE_HOLDER_H_
#define MASTER_STATE_HOLDER_H_

#include <iostream>
#include "HardwareAvailabilityTable.h"

namespace PAN {
class MasterStateHolder {
  private:
    State fsm_state; // Finite State Machine state.
    long number_of_reboots; // Number of software reboots experienced by satellite in its history.
    // This is the RAM copy of this number; the actual copy is stored in EEPROM.
    HAVT havt;
  public:
    enum State { 
      DORMANT_CRUISE,
      INITIALIZATION,
      NORMAL_OPS,
      ECLIPSE,
      SAFE_HOLD,
      DOCKED
    };

    /** Constructor. **/
    MasterStateHolder();

    /** Returns the hardware availability table. **/
    HAVT const& get_havt();

    ///////////////////// State Getters and Setters ////////////////////////
    /** Overrides the current state. **/
    void set_state(State state);

    /** Retrieves the current state of the satellite. */
    State get_state();
    ////////////////////////////////////////////////////////////////////////
};

extern MasterStateHolder msh;
}