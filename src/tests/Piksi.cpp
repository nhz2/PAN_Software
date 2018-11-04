#include <Arduino.h>
#include <HardwareSerial.h>
#include <Piksi.hpp>
using namespace Devices;

Piksi piksi(Serial4);
bool was_setup_successful;

void setup() {
    Serial.begin(9600);
    was_setup_successful = piksi.setup();
}

void loop() {
    Piksi::position_t position;
    // piksi.get_pos_ecef(&position);
    if (piksi.process_buffer()) {
        Serial.printf("Was setup successful?: %d\n", was_setup_successful);
        Serial.printf("Is functional: %d\n", piksi.is_functional());
    }
    // Serial.printf("GPS time of week: %d\n", position.tow);
    // Serial.printf("X position (ECEF): %f\n", position.position[0]);
    // Serial.printf("Y position (ECEF): %f\n", position.position[1]);
    // Serial.printf("Z position (ECEF): %f\n", position.position[2]);
    // Serial.printf("Accuracy: %f\n", position.accuracy);
    delay(1000);
}