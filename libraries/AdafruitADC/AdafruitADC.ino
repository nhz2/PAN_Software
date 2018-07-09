//
// libraries/AdafruitADC/AdafruitADC.ino
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "AdafruitADC.hpp"

ADS1115 adc(Wire, ADS1115_ADDR_VDD, 12);

void setup() {
  pinMode(12, INPUT);
  Serial.begin(9600);
  Wire.begin();
}

void loop() {

  for(unsigned int i = 0; i < 4; i++) {
    bool b;
    int16_t x;
    adc.start_read(i);
    b = adc.end_read(x);
    if(b)
      Serial.println(x);
    else
      Serial.println("Error");
  }
  delay(2000);

}
