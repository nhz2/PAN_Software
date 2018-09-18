#include <Gomspace.hpp>
#include <I2CDevice.hpp>

using namespace PAN::Devices;

Gomspace::Gomspace(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(wire, i2c_addr, 0) {}

// Blank because we can't ever really set up the Gomspace!
Gomspace::dev_setup() {}

Gomspace::dev_is_functional() { return _ping(0x00); }

Gomspace::dev_reset() { _reboot(); }

// Blank because we can't ever really disable the Gomspace!
Gomspace::dev_disable() {}

Gomspace::dev_sc_test() {
    // TODO
    return String("");
}

void Gomspace::_get_hk_2() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x00;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&hk);
    i2c_read(buffer, sizeof(eps_hk_t));
}

void Gomspace::_get_hk_2_vi() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x01;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&hk_vi);
    i2c_read(buffer, sizeof(eps_hk_vi_t));
}

void Gomspace::_get_hk_2_out() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x02;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&hk_out);
    i2c_read(buffer, sizeof(eps_hk_out_t));
}

void Gomspace::_get_hk_2_wdt() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x03;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&hk_wdt);
    i2c_read(buffer, sizeof(eps_hk_wdt_t));
}

void Gomspace::_get_hk_2_basic() {
    uint8_t PORT_BYTE = 0x08;
    uint8_t CMD_TYPE_BYTE = 0x04;
    uint8_t command[2] = {PORT_BYTE, CMD_TYPE_BYTE};
    i2c_write(command, 2);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&hk_basic);
    i2c_read(buffer, sizeof(eps_hk_basic_t));
}

void Gomspace::_set_output(uint8_t output_byte) {
    uint8_t PORT_BYTE = 0x09;
    uint8_t command[2] = {PORT_BYTE, output_byte};
    i2c_write(command, 2);
}

void Gomspace::_set_single_output(uint8_t channel, uint8_t value, int16_t time_delay) {
    uint8_t PORT_BYTE = 0x10;
    uint8_t command[4] = {PORT_BYTE, channel, value, time_delay};
    i2c_write(command, 4);
}

void Gomspace::_set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3) {
    uint8_t PORT_BYTE = 0x11;
    uint8_t command[7] = {PORT_BYTE, voltage1 >> 8, voltage1, voltage2 >> 8, voltage2, voltage3 >> 8, voltage3};
    i2c_write(command, 7);
}

void Gomspace::_set_pv_auto(uint8_t mode) {
    uint8_t PORT_BYTE = 0x12;
    uint8_t command[2] = {PORT_BYTE, mode};
    i2c_write(command, 2);
}

uint8_t* Gomspace::_set_heater(uint8_t cmd, uint8_t header, uint8_t mode) {
    uint8_t PORT_BYTE = 0x13;
    uint8_t command[4] = {PORT_BYTE, cmd, header, mode};
    i2c_write(command, 4);

    static uint8_t buffer[2];
    i2c_read(buffer, 2);
    return buffer;
}

void Gomspace::_get_heater() {
    uint8_t PORT_BYTE = 0x13;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    static uint8_t buffer[2];
    i2c_read(buffer, 2);
    return buffer;
}

void Gomspace::_reset_counters() {
    uint8_t PORT_BYTE = 0x15;
    uint8_t MAGIC_BYTE = 0x42;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_write(command, 2);
}

void Gomspace::_reset_wdt() {
    uint8_t PORT_BYTE = 0x16;
    uint8_t MAGIC_BYTE = 0x78;
    uint8_t command[2] = {PORT_BYTE, MAGIC_BYTE};
    i2c_write(command, 2);
}

void Gomspace::_config_cmd(uint8_t cmd) {
    uint8_t PORT_BYTE = 0x17;
    uint8_t command[2] = {PORT_BYTE, cmd};
    i2c_write(command, 2);
}

void Gomspace::_config_get() {
    uint8_t PORT_BYTE = 0x18;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&config);
    i2c_read(buffer, sizeof(eps_config_t));
}

void Gomspace::_config_set(const eps_config_t &c) {
    uint8_t PORT_BYTE = 0x19;
    uint8_t* config_struct = std::reinterpret_cast<uint8_t*>(&c);
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config_struct, sizeof(eps_config_t));
}

void Gomspace::_hard_reset() {
    uint8_t PORT_BYTE = 0x20;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
}

void Gomspace::_config2_cmd(uint8_t cmd) {
    uint8_t PORT_BYTE = 0x21;
    uint8_t command[2] = {PORT_BYTE, cmd};
    i2c_write(command, 2);
}

void Gomspace::_config2_get() {
    uint8_t PORT_BYTE = 0x22;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    uint8_t* buffer = std::reinterpret_cast<uint8_t*>(&config2);
    i2c_read(buffer, sizeof(eps_config2_t));
}

void Gomspace::_config2_set(const eps_config2_t &c) {
    uint8_t PORT_BYTE = 0x23;
    uint8_t* config2_struct = std::reinterpret_cast<uint8_t*>(&c);
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config2_struct, sizeof(eps_config2_t));
}

void Gomspace::_config3(const eps_config3_t &c) {
    uint8_t PORT_BYTE = 0x25;
    uint8_t* config3_struct = std::reinterpret_cast<uint8_t*>(&c);
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(config3_struct, sizeof(eps_config3_t));
}

bool Gomspace::_ping(uint8_t value) {
    uint8_t PORT_BYTE = 0x01;
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);

    uint8_t response;
    i2c_read(&response, 1);
    return value == response;
}

void Gomspace::_reboot() {
    uint8_t PORT_BYTE = 0x04;
    uint8_t MAGIC[4] = {0x80,0x07,0x80,0x07};
    uint8_t command[1] = {PORT_BYTE};
    i2c_write(command, 1);
    i2c_write(command, 4);
}