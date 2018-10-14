#include <Arduino.h>
#include <HardwareSerial.h>
#include <Piksi.hpp>
using namespace Devices;

Piksi piksi(Serial1);

void setup() {
    piksi.setup();
}

void loop() {
    delay(1000);
}