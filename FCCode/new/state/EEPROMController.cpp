
#include <iostream>
#include <EEPROM.h>
#include "EEPROMController.h"

using namespace PAN;

uint16_t EEPROMController::get_number_of_reboots() {
    uint16_t high_byte = EEPROM.read(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH);
    uint16_t low_byte = EEPROM.read(EEPROMAddresses::NUMBER_OF_REBOOTS_LOW);
    return high_byte << 8 + low_byte;
}

void EEPROMController::increment_number_of_reboots(uint16_t num_reboots) {
    uint16_t num_reboots = get_number_of_reboots();
    num_reboots++;

    uint8_t high_byte = num_reboots >> 8;
    uint8_t low_byte = num_reboots;
    EEPROM.write(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH, high_byte);
    EEPROM.write(EEPROMAddresses::NUMBER_OF_REBOOTS_HIGH, low_byte);
}

Faults::RebootFaults get_reboot_cause() {
    return (Faults::RebootFaults) EEPROM.read(EEPROMAddresses::REBOOT_CAUSE);
}

void clear_reboot_cause() {
    EEPROM.write(EEPROMAddresses::REBOOT_CAUSE, (uint8_t) 0x00);
}

uint32_t EEPROMController::get_hours_since_deployment() {
    uint32_t high_byte  = EEPROM.read(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_HIGH);
    uint32_t two_byte   = EEPROM.read(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_2);
    uint32_t three_byte = EEPROM.read(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_3);
    uint32_t low_byte   = EEPROM.read(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_LOW);
    return high_byte << 24 + two_byte << 16 + three_byte << 8 + low_byte;
}

void EEPROMController::increment_hours_since_deployment() {
    uint32_t num_hours = get_hours_since_deployment();
    num_hours++;

    uint8_t high_byte  = num_hours >> 24;
    uint8_t two_byte   = num_hours >> 16;
    uint8_t three_byte = num_hours >> 8;
    uint8_t low_byte   = num_hours;
    EEPROM.write(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_HIGH, high_byte);
    EEPROM.write(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_2, two_byte);
    EEPROM.write(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_3, three_byte);
    EEPROM.write(EEPROMAddresses::HOURS_SINCE_DEPLOYMENT_LOW, low_byte);
}