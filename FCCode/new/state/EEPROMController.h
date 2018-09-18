#ifndef EEPROM_STATE_H_
#define EEPROM_STATE_H_

#include <iostream>
#include "Faults.h"

namespace PAN {
class EEPROMController {
    public:
      enum EEPROMAddresses
      {
          NUMBER_OF_REBOOTS_HIGH = 0x00,      // Most significant byte of # of satellite reboots
          NUMBER_OF_REBOOTS_LOW = 0x01,       // Least significant byte of # of satellite reboots
          REBOOT_CAUSE = 0x02,                // Byte that contains cause of most recent reboot.
          HOURS_SINCE_DEPLOYMENT_HIGH = 0x03, // Most significant byte of hours since satellite deployment
          HOURS_SINCE_DEPLOYMENT_2 = 0x04, // 2nd most significant byte of hours since satellite deployment
          HOURS_SINCE_DEPLOYMENT_3 = 0x05, // 3rd most significant byte of hours since satellite deployment
          HOURS_SINCE_DEPLOYMENT_LOW = 0x06, // Least significant byte of hours since satellite deployment
      };

      // Getters/setters for number of reboots
      uint16_t get_number_of_reboots();
      void increment_reboots();
      Faults::RebootFaults get_reboot_cause();
      void clear_reboot_cause();

      // Getters/setters for number of hours since satellite deployment
      uint32_t get_hours_since_deployment();
      void increment_hours_since_deployment();
};
}