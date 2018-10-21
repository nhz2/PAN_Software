#include <HardwareSerial.h>
#include "../Devices/I2CDevice.hpp"
#include "Gomspace.hpp"

// Builtins provided by GCC for endian flipping.
#define __bswap_16(x) ((uint16_t)__builtin_bswap16(x))
#define __bswap_32(x) ((uint32_t)__builtin_bswap32(x))

using namespace Devices;

Gomspace::Gomspace(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {
    hk = &hk_data;
    hk_vi = (eps_hk_vi_t*)((uint8_t*)hk + 0);
    hk_out = (eps_hk_out_t*)((uint8_t*)hk_vi + sizeof(eps_hk_vi_t));
    hk_wdt = (eps_hk_wdt_t*)((uint8_t*)hk_out + sizeof(eps_hk_out_t));
    hk_basic = (eps_hk_basic_t*)((uint8_t*)hk_wdt + sizeof(eps_hk_wdt_t));
}

bool Gomspace::setup() {
    bool setup_succeeded = I2CDevice::setup();
    if (!setup_succeeded) return false;
    return true;
}

void Gomspace::reset() {
    I2CDevice::reset();
    reboot();
}

void Gomspace::single_comp_test() {
    I2CDevice::single_comp_test();
    // TODO
}

bool Gomspace::i2c_ping() {
    return ping(0x01);
}

bool Gomspace::get_hk() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x00;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    size_t struct_size = sizeof(eps_hk_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    // FOR DEBUGGING
    // for(uint8_t i = 0; i < sizeof(buffer); i++) {
    //     Serial.printf("%d ", buffer[i]);
    // }
    // Serial.println();
    i2c_finish();

    if (buffer[0] != PORT_BYTE && buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)hk, buffer + 2, struct_size);
        // Flip endianness of all values
        _hk_vi_endian_flip();
        _hk_out_endian_flip();
        _hk_wdt_endian_flip();
        _hk_basic_endian_flip();
        return true;
    }
}

bool Gomspace::get_hk_vi() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x01;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    size_t struct_size = sizeof(eps_hk_vi_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);
    

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)hk_vi, buffer + 2, struct_size);
        _hk_vi_endian_flip();
        return true;
    }
}

bool Gomspace::get_hk_out() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x02;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    size_t struct_size = sizeof(eps_hk_out_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)hk_out, buffer + 2, struct_size);
        _hk_out_endian_flip();
        return true;
    }
}

bool Gomspace::get_hk_wdt() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x03;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    size_t struct_size = sizeof(eps_hk_wdt_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)hk_wdt, buffer + 2, struct_size);
        _hk_wdt_endian_flip();        
        return true;
    }
}

bool Gomspace::get_hk_basic() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x04;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    size_t struct_size = sizeof(eps_hk_basic_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)hk_basic, buffer + 2, struct_size);
        _hk_basic_endian_flip();
        return true;
    }
}

bool Gomspace::set_output(uint8_t output_byte) {
    uint8_t PORT_BYTE = 0x09;
    uint8_t command[2] = {PORT_BYTE, output_byte};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_single_output(uint8_t channel, uint8_t value, int16_t delay) {
    if (channel > 5) return false; // Disallow turning on heater with this command.
    if (value > 1) return false; // Precondition check to avoid radiation bit flips
    // All values must be sent in big-endian order!
    delay = __bswap_16(delay);

    uint8_t PORT_BYTE = 0x0A;
    uint8_t command[5] = {PORT_BYTE, channel, value, (uint8_t) (delay >> 8), (uint8_t) (delay)};
    i2c_begin_transmission();
    i2c_write(command, 5);
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3) {
    // All values must be sent in big-endian order!
    voltage1 = __bswap_16(voltage1);
    voltage2 = __bswap_16(voltage2);
    voltage3 = __bswap_16(voltage3);

    uint8_t PORT_BYTE = 0x0B;
    uint8_t command[7] = {PORT_BYTE, 
        (uint8_t) (voltage1 >> 8), (uint8_t)(voltage1), 
        (uint8_t) (voltage2 >> 8), (uint8_t)(voltage2), 
        (uint8_t) (voltage3 >> 8), (uint8_t)(voltage3)};
    i2c_begin_transmission();
    i2c_write(command, 7);
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_pv_auto(uint8_t mode) {
    if (mode > 1) return false; // Precondition check to avoid radiation bit flips

    uint8_t PORT_BYTE = 0x0C;
    uint8_t command[2] = {PORT_BYTE, mode};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::turn_on_heater() { return _set_heater(true); }
bool Gomspace::turn_off_heater() { return _set_heater(false); }

bool Gomspace::_set_heater(bool mode) {
    if (mode > 1) return false; // Precondition check to avoid radiation bit flips

    uint8_t PORT_BYTE = 0x0D, COMMAND = 0x00;
    uint8_t command[4] = {PORT_BYTE, COMMAND, 1, (uint8_t) mode};
    i2c_begin_transmission();
    i2c_write(command, 4);
    i2c_end_transmission(I2C_NOSTOP);

    uint8_t buffer[4];
    i2c_request_from(4, I2C_STOP);
    i2c_read(buffer, 4);
    if (buffer[0] != PORT_BYTE) return false;
    else if (buffer[1] != 0x00) return false;
    else if (buffer[3] != mode && 1 == (1 | 2)) return false; // If onboard heater was supposed to be turned on/off, 
                                                                   // it better have been reported to be turned on/off.
    else return true;
}

uint8_t Gomspace::get_heater() {
    uint8_t PORT_BYTE = 0x0D;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_end_transmission(I2C_NOSTOP);

    uint8_t buffer[4];
    i2c_request_from(4, I2C_STOP);
    i2c_read(buffer, 4);

    if (buffer[0] != PORT_BYTE || buffer[1] != 0) return 2; // If error occurred, return error code.
    // buffer[2] contains 0 or 1, indicating whether BP4 heater is on. We don't
    // care about this value since we don't have a BP4 pack.
    // buffer[3] contains 0 or 1, indicating whether onboard heater is on.
    return buffer[3];
}

bool Gomspace::reset_counters() {
    uint8_t PORT_BYTE = 0x0F;
    uint8_t MAGIC_BYTE = 0x42;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::reset_wdt() {
    uint8_t PORT_BYTE = 0x10;
    uint8_t MAGIC_BYTE = 0x78;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::config_get() {
    uint8_t PORT_BYTE = 0x12;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_end_transmission(I2C_NOSTOP);
    
    size_t struct_size = sizeof(eps_config_t);
    uint8_t buffer[struct_size + 2];
    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)&gspace_config, buffer + 2, struct_size);
        // Flip endianness
        for(uint8_t i = 0; i < 8; i++) {
            gspace_config.output_initial_on_delay[i] = __bswap_16(gspace_config.output_initial_on_delay[i]);
            gspace_config.output_initial_off_delay[i] = __bswap_16(gspace_config.output_initial_off_delay[i]);
        }
        for(uint8_t i = 0; i < 3; i++) { gspace_config.vboost[i] = __bswap_16(gspace_config.vboost[i]); }
        return true;
    }
}

bool Gomspace::config_set(const eps_config_t &c) {
    uint8_t PORT_BYTE = 0x13;
    uint8_t command[1] = {PORT_BYTE};

    // Flip endianness of all numbers in struct
    uint8_t config_struct[sizeof(eps_config_t)];
    memcpy(config_struct, (uint8_t*) &c, sizeof(eps_config_t));
    eps_config_t* config_struct_ptr = (eps_config_t*) config_struct;
    for(uint8_t i = 0; i < 8; i++) {
        config_struct_ptr->output_initial_on_delay[i] = __bswap_16(config_struct_ptr->output_initial_on_delay[i]);
        config_struct_ptr->output_initial_off_delay[i] = __bswap_16(config_struct_ptr->output_initial_off_delay[i]);
    }
    for(uint8_t i = 0; i < 3; i++) { config_struct_ptr->vboost[i] = __bswap_16(config_struct_ptr->vboost[i]); }
    
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(config_struct, sizeof(eps_config_t));
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::hard_reset() {
    uint8_t PORT_BYTE = 0x14;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::config2_get() {
    uint8_t PORT_BYTE = 0x16;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_end_transmission(I2C_NOSTOP);

    size_t struct_size = sizeof(eps_config2_t);
    uint8_t buffer[struct_size + 2];
    i2c_request_from((struct_size + 2), I2C_STOP);
    i2c_read(buffer, struct_size + 2);

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)&gspace_config2, buffer + 2, struct_size);
        // Flip endianness
        gspace_config2.batt_maxvoltage = __bswap_16(gspace_config2.batt_maxvoltage);
        gspace_config2.batt_safevoltage = __bswap_16(gspace_config2.batt_safevoltage);
        gspace_config2.batt_criticalvoltage = __bswap_16(gspace_config2.batt_criticalvoltage);
        gspace_config2.batt_normalvoltage = __bswap_16(gspace_config2.batt_normalvoltage);
        return true;
    }
}

bool Gomspace::restore_default_config2() {
    uint8_t PORT_BYTE = 0x15; uint8_t COMMAND_BYTE = 0x02;
    uint8_t command[2] = {PORT_BYTE, COMMAND_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    if (!_check_for_error(PORT_BYTE)) return false;
    if (!_config2_confirm()) return false;
    return true;
}

bool Gomspace::config2_set(const eps_config2_t &c) {
    uint8_t PORT_BYTE = 0x17;
    uint8_t command[1] = {PORT_BYTE};

    // Flip endianness of all numbers in struct
    uint8_t config2_struct[sizeof(eps_config2_t)];
    memcpy(config2_struct, (uint8_t*) &c, sizeof(eps_config2_t));
    eps_config2_t* config2_struct_ptr = (eps_config2_t*) config2_struct;
    config2_struct_ptr->batt_criticalvoltage = __bswap_16(config2_struct_ptr->batt_criticalvoltage);
    config2_struct_ptr->batt_maxvoltage = __bswap_16(config2_struct_ptr->batt_maxvoltage);
    config2_struct_ptr->batt_safevoltage = __bswap_16(config2_struct_ptr->batt_safevoltage);
    config2_struct_ptr->batt_normalvoltage = __bswap_16(config2_struct_ptr->batt_normalvoltage);
    
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(config2_struct, sizeof(eps_config2_t));
    i2c_end_transmission(I2C_NOSTOP);
    
    if (!_check_for_error(PORT_BYTE)) return false;
    if (!_config2_confirm()) return false;
    return true;
}

bool Gomspace::_config2_confirm() {
    uint8_t PORT_BYTE = 0x15; uint8_t COMMAND_BYTE = 0x02;
    uint8_t command[2] = {PORT_BYTE, COMMAND_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::ping(uint8_t value) {
    uint8_t PORT_BYTE = 0x01;
    uint8_t command[2] = {PORT_BYTE, value};
    i2c_begin_transmission();
    i2c_write(command, 2);
    i2c_end_transmission(I2C_NOSTOP);

    uint8_t buffer[3];
    i2c_request_from(3, I2C_STOP);
    i2c_read(buffer, 3);

    return (buffer[1] == 0) && (value == buffer[2]);
}

void Gomspace::reboot() {
    uint8_t PORT_BYTE = 0x04;
    uint8_t MAGIC[4] = {0x80,0x07,0x80,0x07};
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(MAGIC, 4);
    i2c_end_transmission(I2C_STOP);
}

bool Gomspace::_check_for_error(uint8_t port_byte) {
    uint8_t buffer[2];
    i2c_request_from(2, I2C_STOP);
    i2c_read(buffer, 2);

    if (buffer[0] == port_byte && buffer[1] == 0) return true;
    return false;
}

void Gomspace::_hk_vi_endian_flip() {
    for(uint8_t i = 0; i < 3; i++) {
        hk_vi->vboost[i] = __bswap_16(hk_vi->vboost[i]);
        hk_vi->curin[i] = __bswap_16(hk_vi->curin[i]);
    }
    hk_vi->vbatt = __bswap_16(hk_vi->vbatt);
    hk_vi->cursun = __bswap_16(hk_vi->cursun);
    hk_vi->cursys = __bswap_16(hk_vi->cursys);
}

void Gomspace::_hk_out_endian_flip() {
    for(uint8_t i = 0; i < 6; i++) {
        hk_out->curout[i] = __bswap_16(hk_out->curout[i]);
        hk_out->latchup[i] = __bswap_16(hk_out->latchup[i]);
    }
    for(uint8_t i = 0; i < 8; i++) {
        hk_out->output_on_delta[i] = __bswap_16(hk_out->output_on_delta[i]);
        hk_out->output_off_delta[i] = __bswap_16(hk_out->output_off_delta[i]);
    }
}

void Gomspace::_hk_wdt_endian_flip() {
    hk_wdt->wdt_i2c_time_left = __bswap_32(hk_wdt->wdt_i2c_time_left);
    hk_wdt->wdt_gnd_time_left = __bswap_32(hk_wdt->wdt_gnd_time_left);
    hk_wdt->counter_wdt_i2c = __bswap_32(hk_wdt->counter_wdt_i2c);
    hk_wdt->counter_wdt_gnd = __bswap_32(hk_wdt->counter_wdt_gnd);
    hk_wdt->counter_wdt_csp[0] = __bswap_32(hk_wdt->counter_wdt_csp[0]);
    hk_wdt->counter_wdt_csp[1] = __bswap_32(hk_wdt->counter_wdt_csp[1]);
}

void Gomspace::_hk_basic_endian_flip() {
    for(uint8_t i = 0; i < 6; i++) {
        hk_basic->temp[i] = __bswap_16(hk_basic->temp[i]);
    }
    hk_basic->counter_boot = __bswap_16(hk_basic->counter_boot);
}