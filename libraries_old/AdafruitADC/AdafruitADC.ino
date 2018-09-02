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

ADS1015 adc(Wire1, ADS1015::ADDR::SDA, 20);

void setup() {
  pinMode(20, INPUT);
  Serial.begin(9600);
  Serial.println("Starting");
  Wire1.begin();
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
