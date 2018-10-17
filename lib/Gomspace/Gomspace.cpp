#include <HardwareSerial.h>
#include "../Devices/I2CDevice.hpp"
#include "Gomspace.hpp"

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
    
    size_t struct_size = sizeof(eps_hk_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

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
    
    size_t struct_size = sizeof(eps_hk_vi_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

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
    
    size_t struct_size = sizeof(eps_hk_out_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

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

    size_t struct_size = sizeof(eps_hk_wdt_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

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
    

    size_t struct_size = sizeof(eps_hk_basic_t);
    uint8_t buffer[struct_size + 2];

    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

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

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_single_output(uint8_t channel, uint8_t value, int16_t time_delay) {
    uint8_t PORT_BYTE = 0x0A;
    uint8_t command[5] = {PORT_BYTE, channel, value, (uint8_t) time_delay, (uint8_t)(time_delay >> 8)}; // Order of time-delay bytes flipped due to endianness!
    i2c_begin_transmission();
    i2c_write(command, 5);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3) {
    uint8_t PORT_BYTE = 0x0B;
    uint8_t command[7] = {PORT_BYTE, 
        (uint8_t) voltage1, (uint8_t)(voltage1 >> 8), 
        (uint8_t) voltage2, (uint8_t)(voltage2 >> 8), 
        (uint8_t) voltage3, (uint8_t)(voltage3 >> 8)};
    i2c_begin_transmission();
    i2c_write(command, 7);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_pv_auto(uint8_t mode) {
    uint8_t PORT_BYTE = 0x0C;
    uint8_t command[2] = {PORT_BYTE, mode};
    i2c_begin_transmission();
    i2c_write(command, 2);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::set_heater(uint8_t heater, uint8_t mode) {
    uint8_t PORT_BYTE = 0x0D, COMMAND = 0x00;
    uint8_t command[4] = {PORT_BYTE, COMMAND, heater, mode};
    i2c_begin_transmission();
    i2c_write(command, 4);  

    return _check_for_error(PORT_BYTE);
}

uint8_t Gomspace::get_heater() {
    uint8_t PORT_BYTE = 0x0D;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);

    uint8_t buffer[4];
    i2c_request_from(4, I2C_NOSTOP);
    i2c_read(buffer, 4);
    i2c_end_transmission();

    if (buffer[1] != 0) return 4; // If error occurred, return error code.
    // buffer[2] contains 0 or 1, indicating whether BP4 heater is on.
    // buffer[3] contains 0 or 1, indicating whether onboard heater is on.
    return buffer[2] + 2 * buffer[3];
}

bool Gomspace::reset_counters() {
    uint8_t PORT_BYTE = 0x0F;
    uint8_t MAGIC_BYTE = 0x42;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::reset_wdt() {
    uint8_t PORT_BYTE = 0x10;
    uint8_t MAGIC_BYTE = 0x78;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 2);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::restore_default_config() {
    uint8_t PORT_BYTE = 0x11, COMMAND = 0x01;
    uint8_t command[2] = {PORT_BYTE, COMMAND};
    i2c_begin_transmission();
    i2c_write(command, 2);
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::config_get() {
    uint8_t PORT_BYTE = 0x12;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    
    size_t struct_size = sizeof(eps_config_t);
    uint8_t buffer[struct_size + 2];
    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)&gspace_config, buffer + 2, struct_size);
        // Flip endianness
        for(uint8_t i = 0; i < 8; i++) {
            gspace_config.output_initial_on_delay[i] = _flip_endian(gspace_config.output_initial_on_delay[i]);
            gspace_config.output_initial_off_delay[i] = _flip_endian(gspace_config.output_initial_off_delay[i]);
        }
        for(uint8_t i = 0; i < 3; i++) {
            gspace_config.vboost[i] = _flip_endian(gspace_config.vboost[i]);
        }
        return true;
    }
}

bool Gomspace::config_set(const eps_config_t &c) {
    uint8_t PORT_BYTE = 0x13;
    uint8_t* config_struct = (uint8_t*)&c;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(config_struct, sizeof(eps_config_t));
    

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::hard_reset() {
    uint8_t PORT_BYTE = 0x14;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::restore_default_config2() {
    uint8_t PORT_BYTE = 0x15, COMMAND = 0x01;
    uint8_t command[2] = {PORT_BYTE, COMMAND};
    i2c_begin_transmission();
    i2c_write(command, 2);

    return _check_for_error(PORT_BYTE);
}

bool Gomspace::config2_get() {
    uint8_t PORT_BYTE = 0x16;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);

    size_t struct_size = sizeof(eps_config2_t);
    uint8_t buffer[struct_size + 2];
    i2c_request_from((struct_size + 2), I2C_NOSTOP);
    i2c_read(buffer, struct_size + 2);
    i2c_end_transmission();

    if (buffer[1] != 0) return false;
    else {
        memcpy((uint8_t*)&gspace_config2, buffer + 2, struct_size);
        // Flip endianness
        gspace_config2.batt_maxvoltage = _flip_endian(gspace_config2.batt_maxvoltage);
        gspace_config2.batt_safevoltage = _flip_endian(gspace_config2.batt_safevoltage);
        gspace_config2.batt_criticalvoltage = _flip_endian(gspace_config2.batt_criticalvoltage);
        gspace_config2.batt_normalvoltage = _flip_endian(gspace_config2.batt_normalvoltage);
        return true;
    }
}

bool Gomspace::config2_set(const eps_config2_t &c) {
    uint8_t PORT_BYTE = 0x17;
    uint8_t* config2_struct = (uint8_t*)&c;
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(config2_struct, sizeof(eps_config2_t));
    
    return _check_for_error(PORT_BYTE);
}

bool Gomspace::ping(uint8_t value) {
    uint8_t PORT_BYTE = 0x01;
    uint8_t command[2] = {PORT_BYTE, value};
    i2c_begin_transmission();
    i2c_write(command, 2);

    uint8_t buffer[3];
    i2c_request_from(3, I2C_NOSTOP);
    i2c_read(buffer, 3);
    i2c_end_transmission();

    return (buffer[1] == 0) && (value == buffer[2]);
}

void Gomspace::reboot() {
    uint8_t PORT_BYTE = 0x04;
    uint8_t MAGIC[4] = {0x80,0x07,0x80,0x07};
    uint8_t command[1] = {PORT_BYTE};
    i2c_begin_transmission();
    i2c_write(command, 1);
    i2c_write(MAGIC, 4);
    i2c_end_transmission();
}

bool Gomspace::_check_for_error(uint8_t port_byte) {
    uint8_t buffer[2];
    i2c_request_from(2, I2C_NOSTOP);
    i2c_read(buffer, 2);
    i2c_end_transmission();

    if (buffer[0] == port_byte && buffer[1] == 0) return true;
    return false;
}

uint16_t Gomspace::_flip_endian(uint16_t n) {
    uint16_t b0 = (n & 0x00ff) << 8u;
    uint16_t b1 = (n & 0xff00) >> 8u;

    return b0 | b1;
}

int16_t Gomspace::_flip_endian(int16_t n) {
    return (int16_t) _flip_endian((uint16_t) n);
}

uint32_t Gomspace::_flip_endian(uint32_t n) {
    uint32_t b0 = (n & 0x000000ff) << 24u;
    uint32_t b1 = (n & 0x0000ff00) << 8u;
    uint32_t b2 = (n & 0x00ff0000) >> 8u;
    uint32_t b3 = (n & 0xff000000) >> 24u;

    return b0 | b1 | b2 | b3;
}

void Gomspace::_hk_vi_endian_flip() {
    for(uint8_t i = 0; i < 3; i++) {
        hk_vi->vboost[i] = _flip_endian(hk_vi->vboost[i]);
        hk_vi->curin[i] = _flip_endian(hk_vi->curin[i]);
    }
    hk_vi->vbatt = _flip_endian(hk_vi->vbatt);
    hk_vi->cursun = _flip_endian(hk_vi->cursun);
    hk_vi->cursys = _flip_endian(hk_vi->cursys);
}

void Gomspace::_hk_out_endian_flip() {
    for(uint8_t i = 0; i < 6; i++) {
        hk_out->curout[i] = _flip_endian(hk_out->curout[i]);
        hk_out->latchup[i] = _flip_endian(hk_out->latchup[i]);
    }
    for(uint8_t i = 0; i < 8; i++) {
        hk_out->output_on_delta[i] = _flip_endian(hk_out->output_on_delta[i]);
        hk_out->output_off_delta[i] = _flip_endian(hk_out->output_off_delta[i]);
    }
}

void Gomspace::_hk_wdt_endian_flip() {
    hk_wdt->wdt_i2c_time_left = _flip_endian(hk_wdt->wdt_i2c_time_left);
    hk_wdt->wdt_gnd_time_left = _flip_endian(hk_wdt->wdt_gnd_time_left);
    hk_wdt->counter_wdt_i2c = _flip_endian(hk_wdt->counter_wdt_i2c);
    hk_wdt->counter_wdt_gnd = _flip_endian(hk_wdt->counter_wdt_gnd);
    hk_wdt->counter_wdt_csp[0] = _flip_endian(hk_wdt->counter_wdt_csp[0]);
    hk_wdt->counter_wdt_csp[1] = _flip_endian(hk_wdt->counter_wdt_csp[1]);
}

void Gomspace::_hk_basic_endian_flip() {
    for(uint8_t i = 0; i < 6; i++) {
        hk_basic->temp[i] = _flip_endian(hk_basic->temp[i]);
    }
    hk_basic->counter_boot = _flip_endian(hk_basic->counter_boot);
}