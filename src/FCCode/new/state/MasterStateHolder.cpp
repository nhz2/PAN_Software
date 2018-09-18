#include <iostream>
#include <EEPROM.h>
#include "MasterStateHolder.h"

using namespace PAN;

MasterStateHolder::MasterStateHolder() {
    fsm_state = DORMANT_CRUISE;

    auto device_list = havt.get_device_list();
    // Set up hardware table to indicate resources as unavailable, by default
    for (int i = 0; i < device_list.size(); i++) {
        havt.set_device_responding(device_list.at(i)) = false;
        havt.set_device_functional(device_list.at(i)) = false;
    }

    // Load and rewrite number of reboots that have occurred from EEPROM. The
    // number of reboots is stored in the first two bytes of the EEPROM in little-endian
    // format.
    number_of_reboots = EEPROM.read(0x00) << 8 + EEPROM.read(0x01);
    number_of_reboots++;
    EEPROM.write(0x00, number_of_reboots >> 8);
    EEPROM.write(0x01, number_of_reboots);
}

HAVT const &MasterStateHolder::get_havt()
{
    return havt;
}

void MasterStateHolder::set_state(MasterStateHolder::State state) {
    fsm_state = state;
}

State MasterStateHolder::get_state() {
    return fsm_state;
}