#include "../Devices/I2CDevice.hpp"
#include "Gomspace.hpp"

using namespace Devices;

Gomspace::Gomspace(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {}

bool Gomspace::setup() {
    bool setup_succeeded = I2CDevice::setup();
    if (!setup_succeeded) return false;

#ifndef TESTING
    gspace_config.ppt_mode = 1;
    gspace_config.battheater_mode = 1;
    gspace_config.battheater_low = -20;
    gspace_config.battheater_low = 50;
    for(uint8_t i = 0; i < 8; i++) {
        gspace_config.output_normal_value[i] = 1; // Form needs to be defined
        gspace_config.output_safe_value[i] = 0; // Form needs to be defined
        gspace_config.output_initial_on_delay[i] = 0; // Form needs to be defined
        gspace_config.output_initial_off_delay[i] = 0; // Form needs to be defined
    }
    config_set(gspace_config);

    gspace_config2.batt_maxvoltage = 0; // Value needs to be defined
    gspace_config2.batt_safevoltage = 0; // Value needs to be defined
    gspace_config2.batt_criticalvoltage = 0; // Value needs to be defined
    gspace_config2.batt_normalvoltage = 0; // Value needs to be defined
    config2_set(gspace_config2);

    // Interaction pattern for cmd3 needs to be determined
    gspace_config3.version = 1;
    gspace_config3.cmd = 4;
    gspace_config3.length = 11;
    gspace_config3.flags = 0;
    for(uint8_t i = 0; i < 8; i++) {
        gspace_config3.cur_lim[i] = 0; // Value and form needs to be defined
    }
    gspace_config3.cspwdt_address[0] = 0; // Does this need to be set if we're using I2C watch dogs?
    gspace_config3.cspwdt_address[1] = 0; // Does this need to be set if we're using I2C watch dogs?
    gspace_config3.cspwdt_channel[0] = 0; // Does this need to be set if we're using I2C watch dogs?
    gspace_config3.cspwdt_channel[1] = 0; // Does this need to be set if we're using I2C watch dogs?
#endif

    return true;
}

void Gomspace::reset() {
    I2CDevice::reset();
    reboot(); 
}

void Gomspace::single_comp_test() {
    // TODO
}

bool Gomspace::i2c_ping() {
    return ping(0x01);
}

const Gomspace::eps_hk_t &Gomspace::get_hk_2() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x00;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = (uint8_t*)&hk;
    i2c_read(buffer, sizeof(eps_hk_t));

    return hk;
}

void Gomspace::set_output(uint8_t output_byte) {
    uint8_t PORT_BYTE = 0x09;
    uint8_t command[2] = {PORT_BYTE, output_byte};
    i2c_write(command, 2);
}

void Gomspace::set_single_output(uint8_t channel, uint8_t value, int16_t time_delay) {
    uint8_t PORT_BYTE = 0x10;
    uint8_t command[5] = {PORT_BYTE, channel, value, (uint8_t)(time_delay >> 8), (uint8_t) time_delay};
    i2c_write(command, 5);
}

void Gomspace::set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3) {
    uint8_t PORT_BYTE = 0x11;
    uint8_t command[7] = {PORT_BYTE, 
        (uint8_t)(voltage1 >> 8), (uint8_t) voltage1, 
        (uint8_t)(voltage2 >> 8), (uint8_t) voltage2, 
        (uint8_t)(voltage3 >> 8), (uint8_t) voltage3};
    i2c_write(command, 7);
}

void Gomspace::set_pv_auto(uint8_t mode) {
    uint8_t PORT_BYTE = 0x12;
    uint8_t command[2] = {PORT_BYTE, mode};
    i2c_write(command, 2);
}

uint8_t* Gomspace::set_heater(uint8_t cmd, uint8_t header, uint8_t mode) {
    uint8_t PORT_BYTE = 0x13;
    uint8_t command[4] = {PORT_BYTE, cmd, header, mode};
    i2c_write(command, 4);

    static uint8_t buffer[2];
    i2c_read(buffer, 2);
    return buffer;
}

uint8_t* Gomspace::get_heater() {
    uint8_t PORT_BYTE = 0x13;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    static uint8_t buffer[2];
    i2c_read(buffer, 2);
    return buffer;
}

void Gomspace::reset_counters() {
    uint8_t PORT_BYTE = 0x15;
    uint8_t MAGIC_BYTE = 0x42;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_write(command, 2);
}

void Gomspace::reset_wdt() {
    uint8_t PORT_BYTE = 0x16;
    uint8_t MAGIC_BYTE = 0x78;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_write(command, 2);
}

void Gomspace::config_cmd(uint8_t cmd) {
    uint8_t PORT_BYTE = 0x17;
    uint8_t command[2] = {PORT_BYTE, cmd};
    i2c_write(command, 2);
}

void Gomspace::config_get() {
    uint8_t PORT_BYTE = 0x18;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    uint8_t* buffer = (uint8_t*)&gspace_config;
    i2c_read(buffer, sizeof(eps_config_t));
}

void Gomspace::config_set(const eps_config_t &c) {
    uint8_t PORT_BYTE = 0x19;
    uint8_t* config_struct = (uint8_t*)&c;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config_struct, sizeof(eps_config_t));
}

void Gomspace::hard_reset() {
    uint8_t PORT_BYTE = 0x20;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
}

void Gomspace::config2_cmd(uint8_t cmd) {
    uint8_t PORT_BYTE = 0x21;
    uint8_t command[2] = {PORT_BYTE, cmd};
    i2c_write(command, 2);
}

void Gomspace::config2_get() {
    uint8_t PORT_BYTE = 0x22;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    uint8_t* buffer = (uint8_t*)&gspace_config2;
    i2c_read(buffer, sizeof(eps_config2_t));
}

void Gomspace::config2_set(const eps_config2_t &c) {
    uint8_t PORT_BYTE = 0x23;
    uint8_t* config2_struct = (uint8_t*)&c;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config2_struct, sizeof(eps_config2_t));
}

void Gomspace::config3(const eps_config3_t &c) {
    uint8_t PORT_BYTE = 0x25;
    uint8_t* config3_struct = (uint8_t*)&c;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config3_struct, sizeof(eps_config3_t));
}

bool Gomspace::ping(uint8_t value) {
    uint8_t PORT_BYTE = 0x01;
    uint8_t command[2] = {PORT_BYTE, value};
    i2c_write(command, 2);

    uint8_t response;
    i2c_read(&response, 1);
    return value == response;
}

void Gomspace::reboot() {
    uint8_t PORT_BYTE = 0x04;
    uint8_t MAGIC[4] = {0x80,0x07,0x80,0x07};
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(MAGIC, 4);
}
