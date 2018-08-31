#ifndef EEPROM_STATE_H_
#define EEPROM_STATE_H_

#include <iostream>
#include "Faults.h"

namespace PAN {
class EEPROMController {
    public:
      enum EEPROMAddresses
      {
          NUMBER_OF_REBOOTS_HIGH = 0x00, // Most significant byte of # of satellite reboots
          NUMBER_OF_REBOOTS_LOW = 0x01,  // Most significant byte of # of satellite reboots
          REBOOT_CAUSE = 0x02, // Byte that contains cause of most recent reboot.
      };

      // Getters/setters for number of reboots
      uint16_t get_number_of_reboots();
      void set_number_of_reboots(uint16_t num_reboots);
      Faults::RebootFaults get_reboot_cause();
      void clear_reboot_cause();
};
}