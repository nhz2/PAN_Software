#include <Arduino.h>
#include <SPI.h>
#include <MCP4162.hpp>

using namespace Devices;

MCP4162::MCP4162(SPIClass &spi_bus, uint8_t slave_select) : spi(spi_bus) {
    ss = slave_select;
    _min_resistance = 125;
    _max_resistance = 5000;

    pinMode(ss, OUTPUT);
    spi.begin();
    spi.setBitOrder(MSBFIRST); // MSBFIRST required by MCP4162.
}

bool MCP4162::dev_setup() {
    return dev_is_functional();
}

bool MCP4162::dev_is_functional() {
    // There's no way for us to verify, in hardware, whether the 
    // digital pot is working or not. So we just return true.
    return true;
}

// These functions are not implemented, because the device is extremely simple
// and actually cannot be reset/disabled.
void MCP4162::dev_reset() {}
void MCP4162::dev_disable() {}

String MCP4162::dev_sc_test() {
    return String(_wiper_setting);
}

void MCP4162::set_wiper(uint8_t value) {
    _wiper_setting = value;
    digitalWrite(ss, LOW);
    spi.transfer(0);
    spi.transfer(value);
    digitalWrite(ss, HIGH);
}

void MCP4162::set_temperature(float t) {
    // TODO
    // Use a lookup table to recalibrate max and min resistances
    temperature = t;
    _max_resistance = 0;
    _min_resistance = 0;
    _delta_resistance = (_max_resistance - _min_resistance) / 256;
}

void MCP4162::set_resistance(float r) {
    if (r < _min_resistance || r > _max_resistance) return;
    
    uint8_t wiper = (r - _min_resistance) / _delta_resistance - 1;
    set_wiper(wiper);
    resistance = r;
}

float MCP4162::get_resistance() {
    return resistance;
}