#include <Arduino.h>
#include <i2c_t3_pan/i2c_t3_pan.h>
#include <Gomspace.hpp>
using namespace Devices;

#define TESTING true

Gomspace gs(Wire1, Gomspace::ADDRESS);
bool setup_result;

void setup() {
    Serial.begin(9600);
    setup_result = gs.setup();
    
}

void loop() {
    if (!setup_result) Serial.println("Failed to setup device");
    Gomspace::eps_hk_t hk = gs.get_hk_2();
    Serial.printf("Current battery level: %d\n", hk.vbatt);
    delay(1000);
}