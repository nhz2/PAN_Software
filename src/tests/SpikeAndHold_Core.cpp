#include <Arduino.h>

void setup() {
}

void loop() {
    uint8_t pin = 3; // Valve 1
    digitalWrite(pin, HIGH);
    delay(50);
    digitalWrite(pin, LOW);
}