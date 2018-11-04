#include <Arduino.h>

void setup() {
}

void loop() {
    uint8_t pin = 3; // Valve 1
    uint8_t pin2 = 4; // Valve 1
    digitalWrite(pin, HIGH);
    delay(2);
    digitalWrite(pin2, HIGH);
    delay(50);
    digitalWrite(pin, LOW);
    digitalWrite(pin2, LOW);
}