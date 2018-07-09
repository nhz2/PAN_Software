//
// libraries/PololuIMU/PololuIMU.ino
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "PololuIMU.hpp"

LIS3MDL mag(Wire, LIS3MDL_ADDR_LOW);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mag.i2c_set_timeout(1000);
}

void loop() {
  if(mag.read()) {
    Serial.println(String(mag.x()) + "," + String(mag.y()) + "," + String(mag.z()));
  } else {
    Serial.println("Error");
  }
  delay(2000);
}
