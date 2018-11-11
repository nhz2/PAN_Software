#include <Arduino.h>
#include <SPI.h>
#include <ADIS16470.h>
using namespace Devices;

uint8_t cs = 0;
uint8_t dr = 0;
uint8_t rst = 0;
ADIS16470 gyro(cs, dr, rst, SPI);

void setup() {

}

void loop() {

}