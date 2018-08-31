
#include <iostream>
#include <EEPROM.h>
#include "EEPROMController.h"

using namespace PAN;

uint16_t EEPROMController::get_number_of_reboots() {
    uint16_t high_byte = EEPROM.read(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH);
    uint16_t low_byte = EEPROM.read(EEPROMAddresses::NUMBER_OF_REBOOTS_LOW);
    return high_byte << 8 + low_byte;
}

void EEPROMController::set_number_of_reboots(uint16_t num_reboots) {
    uint8_t high_byte = num_reboots >> 8;
    uint8_t low_byte = num_reboots;
    EEPROM.write(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH, high_byte);
    EEPROM.write(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH, low_byte);
}