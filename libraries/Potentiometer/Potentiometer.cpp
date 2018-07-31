//
// libraries/Potentiometer/Potentiometer.cpp
//
// Created by Nathan Zimmerberg (nhz2@cornell.edu) on 07/30/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//
#include "Potentiometer.hpp"

AD5254::AD5254(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {
  for(int i = 0; i < 3; i++)
    this->rdac[i] = 0;
}

void AD5254::set_rdac(uint8_t rdac0, uint8_t rdac1, uint8_t rdac2){
  rdac[0]=rdac0;
  rdac[1]=rdac1;
  rdac[2]=rdac2;
}
uint8_t AD5254::get_rdac0(){
  return rdac[0];
}
uint8_t AD5254::get_rdac1(){
  return rdac[1];
}
uint8_t AD5254::get_rdac2(){
  return rdac[2];
}

bool AD5254::read_block(){
  i2c_pop_errors();

  // Request data
  uint8_t data[3];
  i2c_write_byte(0b10000000);
  i2c_read_bytes(data, 3);
  if(i2c_pop_errors())
    return false;
  // Successful read
  for (int i = 0; i < 3; i++) {
    rdac[i]=data[i];
  }
  return true;
}

bool AD5254::write_block(){
  uint8_t data[]={
    0x00,
    rdac[0],
    rdac[1],
    rdac[2]
  };
  i2c_pop_errors();
  i2c_write_bytes(data,4);
  // Process potential configuration errors
  if(i2c_pop_errors())
    return false;
  return true;
}

bool AD5254::write_noblock(){
  uint8_t data[]={
    0x00,
    rdac[0],
    rdac[1],
    rdac[2]
  };
  i2c_begin_transmission();
  i2c_write(data,4);
  i2c_send_transmission();
  return true;
}
