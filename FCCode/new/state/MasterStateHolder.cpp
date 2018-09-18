#include <iostream>
#include <EEPROM.h>
#include "MasterStateHolder.h"
#include "HardwareAvailabilityTable.h"

using namespace PAN;

MasterStateHolder::MasterStateHolder() {
    fsm_state = DORMANT_CRUISE;

    auto device_list = havt.get_device_list();
    // Set up hardware table to indicate resources as unavailable, by default
    for (int i = 0; i < device_list.size(); i++) {
        havt.set_device_responding(device_list.at(i)) = false;
        havt.set_device_functional(device_list.at(i)) = false;
    }

    // Load and rewrite number of reboots that have occurred from EEPROM. The number of 
    // reboots is stored in the first two bytes of the EEPROM in little-endian format.
    eeprom.increment_number_of_reboots(number_of_reboots);
    number_of_reboots = eeprom.get_number_of_reboots();

    // If the current initialization is due to a reboot fault, store the fault.
    // TODO
}

HardwareAvailabilityTable const &MasterStateHolder::get_havt()
{
    return havt;
}

void MasterStateHolder::set_state(MasterStateHolder::State state) {
    fsm_state = state;
}

MasterStateHolder::State MasterStateHolder::get_state() {
    return fsm_state;
}