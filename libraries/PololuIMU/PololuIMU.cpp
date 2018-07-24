//
// libraries/PololuIMU/PololuIMU.cpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
// Gyro and Accelerometer drivers created by Nathan Zimmerberg (nhz2@cornell.edu) on 07/10/2018
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#include "PololuIMU.hpp"

// LIS3MDL class implementation ----------------------------------------------------

LIS3MDL::LIS3MDL(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {
  for(int i = 0; i < 3; i++)
    this->mag[i] = 0;
  this->sample_frequency = LIS3MDL_SF_0_625;
  this->xy_performance = LIS3MDL_PERF_LOW;
  this->z_performance = LIS3MDL_PERF_LOW;
  this->scale = LIS3MDL_SCALE_PM4G;
  this->should_reset = true;
}

void LIS3MDL::set_sample_frequency(uint8_t sample_frequency) {
  this->sample_frequency = sample_frequency;
}

uint8_t LIS3MDL::get_sample_frequency() const {
  return this->sample_frequency;
}

void LIS3MDL::set_xy_performance(uint8_t xy_performance) {
  this->xy_performance = xy_performance;
}

uint8_t LIS3MDL::get_xy_performance() const {
  return this->xy_performance;
}

void LIS3MDL::set_z_performance(uint8_t z_performance) {
  this->z_performance = z_performance;
}

uint8_t LIS3MDL::get_z_performance() const {
  return this->z_performance;
}

void LIS3MDL::set_scale(uint8_t scale) {
  this->scale = scale;
}

uint8_t LIS3MDL::get_scale() const {
  return this->scale;
}

int16_t LIS3MDL::x() const {
  return this->mag[0];
}

int16_t LIS3MDL::y() const {
  return this->mag[1];
}

int16_t LIS3MDL::z() const {
  return this->mag[2];
}

bool LIS3MDL::read() {
  // Update configuration if needed
  if(this->should_reset) {
    uint8_t arr[] = {
      LIS3MDL_REG_CTRL1, (uint8_t) ((this->xy_performance << 5) | this->sample_frequency),
      LIS3MDL_REG_CTRL2, this->scale,
      LIS3MDL_REG_CTRL3, 0,
      LIS3MDL_REG_CTRL4, (uint8_t) (this->z_performance << 2),
      LIS3MDL_REG_CTRL5, 0
    };
    for(int i = 0; i < 10; i += 2)
      i2c_write_bytes(&arr[i], 2);
    // Process potential configuration errors
    if(i2c_pop_errors())
      return false;
  }
  // Set register and request data
  uint8_t data[6];
  i2c_write_byte(LIS3MDL_REG_OUT_XL | 0x80);
  i2c_read_bytes(data, 6);
  if(i2c_pop_errors())
    return false;
  // Successful read
  for(int i = 0; i < 3; i++)
    this->mag[i] = (int16_t) ((data[2 * i + 1] << 8) | data[2 * i]);
  this->should_reset = false;
  return true;
}

LSM6DS33::LSM6DS33(i2c_t3 &i2c_wire, uint8_t i2c_addr, uint8_t g_odr, uint8_t xl_odr) : I2CDevice(i2c_wire, i2c_addr, 0) {
  this->powered_down= true;
  this->g_odr= g_odr;
  this->xl_odr= xl_odr;
  this->g_fs= 0;
  this->drdy_mask= true;
  this->bdu= true;
  this->whoami= 0;
  this->status= 0;
  this->g_burst= false;
  }

void LSM6DS33::writeReg(regAddr reg, uint8_t value){
    uint8_t data[]= {reg,value};
    i2c_write_bytes(data,2);
  }

uint8_t LSM6DS33::readReg(regAddr reg){
    uint8_t value;
    i2c_write_byte(reg);
    i2c_read_byte(value);
    return value;
  }

void LSM6DS33::gyroselftest_helper(uint8_t testval){
  writeReg(CTRL1_XL, 0x00);
  writeReg(CTRL2_G, 0x5C);
  writeReg(CTRL3_C, 0x44);
  writeReg(CTRL4_C, 0x00);
  writeReg(CTRL5_C, 0x00);
  writeReg(CTRL6_C, 0x00);
  writeReg(CTRL7_G, 0x00);
  writeReg(CTRL8_XL, 0x00);
  writeReg(CTRL9_XL, 0x00);
  writeReg(CTRL10_C, 0x38);

  delay(800);
  for(int j= 0; j<2; j++){
    while (!((2) & readReg(STATUS_REG))){}
    readReg(OUTX_H_G);
    readReg(OUTX_L_G);
    readReg(OUTY_H_G);
    readReg(OUTY_L_G);
    readReg(OUTZ_H_G);
    readReg(OUTZ_L_G);

    for (int i=0; i<5; i++){
      while (!((2) & readReg(STATUS_REG))){}
      Serial.println((((int)readReg(OUTX_H_G))<<8) + (int)readReg(OUTX_L_G));
      Serial.println((((int)readReg(OUTY_H_G))<<8) + (int)readReg(OUTY_L_G));
      Serial.println((((int)readReg(OUTZ_H_G))<<8) + (int)readReg(OUTZ_L_G));
    }

    writeReg(CTRL5_C, testval);
    delay(60);
  }

  writeReg(CTRL2_G, 0x00);
  writeReg(CTRL5_C, 0x00);

}

void LSM6DS33::xlselftest_helper(uint8_t testval){
  writeReg(CTRL1_XL, 0x30);
  writeReg(CTRL2_G, 0x00);
  writeReg(CTRL3_C, 0x44);
  writeReg(CTRL4_C, 0x00);
  writeReg(CTRL5_C, 0x00);
  writeReg(CTRL6_C, 0x00);
  writeReg(CTRL7_G, 0x00);
  writeReg(CTRL8_XL, 0x00);
  writeReg(CTRL9_XL, 0x38);
  writeReg(CTRL10_C, 0x00);

  delay(200);
  for(int j= 0; j<2; j++){
    while (!((1) & readReg(STATUS_REG))){}
    readReg(OUTX_H_XL);
    readReg(OUTX_L_XL);
    readReg(OUTY_H_XL);
    readReg(OUTY_L_XL);
    readReg(OUTZ_H_XL);
    readReg(OUTZ_L_XL);

    for (int i=0; i<5; i++){
      while (!((1) & readReg(STATUS_REG))){}
      Serial.println((((int)readReg(OUTX_H_XL))<<8) + (int)readReg(OUTX_L_XL));
      Serial.println((((int)readReg(OUTY_H_XL))<<8) + (int)readReg(OUTY_L_XL));
      Serial.println((((int)readReg(OUTZ_H_XL))<<8) + (int)readReg(OUTZ_L_XL));
    }

    writeReg(CTRL5_C, testval);
    delay(200);
  }

  writeReg(CTRL1_XL, 0x00);
  writeReg(CTRL5_C, 0x00);
}

void LSM6DS33::self_test() {
  gyroselftest_helper(0x04);
  gyroselftest_helper(0x0C);
  xlselftest_helper(0x01);
  xlselftest_helper(0x02);
}

bool LSM6DS33::power_up(){
  uint8_t data[]= {
    CTRL1_XL,
    (uint8_t)(xl_odr<<4),
    (uint8_t)((g_odr<<4)+ ( g_fs?((g_fs-1)<<2): 2 )),
    (uint8_t)((bdu<<6)+4),
    (uint8_t)((drdy_mask<<3)),
    (uint8_t)((g_burst<<6))
  };
  i2c_pop_errors();
  i2c_write_bytes(data,5);
  // Process potential configuration errors
  if(i2c_pop_errors())
    return false;
  powered_down= false;
  return true;
}

bool LSM6DS33::power_down(){
  uint8_t data[]= {
    CTRL1_XL,
    0x00,
    (uint8_t)(0x00+ ( g_fs?((g_fs-1)<<2): 2 )),
  };
  i2c_pop_errors();
  i2c_write_bytes(data,3);
  // Process potential configuration errors
  if(i2c_pop_errors())
    return false;
  powered_down= true;
  return true;
}

bool LSM6DS33::reboot(){
  i2c_pop_errors();
  writeReg(CTRL2_G,0x00);

  writeReg(CTRL9_XL,0x38);
  writeReg(CTRL1_XL,0x00);
  writeReg(CTRL6_C,0x00);
  writeReg(CTRL1_XL,0x60);
  writeReg(CTRL3_C,0x84);
  // Process potential configuration errors
  if(i2c_pop_errors())
    return false;
  powered_down= true;
  return true;
}

bool LSM6DS33::reset(){
  i2c_pop_errors();
  writeReg(CTRL2_G,0x00);

  writeReg(CTRL9_XL,0x38);
  writeReg(CTRL1_XL,0x00);
  writeReg(CTRL6_C,0x00);
  writeReg(CTRL1_XL,0x60);
  writeReg(CTRL3_C,0x05);
  // Process potential configuration errors
  if(i2c_pop_errors())
    return false;
  powered_down= true;
  return true;
}

bool LSM6DS33::update_cfg(){
  if (powered_down)
    return true;
  return power_up();
}

bool LSM6DS33::setup_gburst(){
  i2c_pop_errors();
  writeReg(CTRL5_C,1<<6);
  i2c_write_byte(OUTX_L_G);
  if(i2c_pop_errors())
    return false;
  g_burst= true;

  return true;
}

bool LSM6DS33::stop_gburst(){
  i2c_pop_errors();
  writeReg(CTRL5_C,0x00);
  if(i2c_pop_errors())
    return false;
  g_burst= false;
  return true;
}

bool LSM6DS33::read_gburst(){
  i2c_pop_errors();
  uint8_t data[6];
  i2c_read_bytes(data, 6);
  if(i2c_pop_errors())
    return false;
  // Successful read
  for (int i=0; i<3; i++){
    gout[i]= (((int16_t)data[2*i+1])<<8) + ((int16_t)data[2*i]);
  }
  return true;
}

bool LSM6DS33::read(){
  i2c_pop_errors();
  whoami= readReg(WHO_AM_I);

  // Set register and request data
  uint8_t data[16];
  i2c_write_byte(STATUS_REG);
  i2c_read_bytes(data, 16);
  if(i2c_pop_errors())
    return false;
  // Successful read
  status= data[0];
  tempout= (((int16_t)data[2])<<8) + ((int16_t)data[1]);
  for (int i=0; i<3; i++){
    gout[i]= (((int16_t)data[2*i+4])<<8) + ((int16_t)data[2*i+3]);
  }
  for (int i=0; i<3; i++){
    xlout[i]= (((int16_t)data[2*i+10])<<8) + ((int16_t)data[2*i+9]);
  }
  return true;
}

bool LSM6DS33::get_power_state() const{
  return !powered_down;
}

void LSM6DS33::set_g_odr(uint8_t odr){
  g_odr= odr;
}
uint8_t LSM6DS33::get_g_odr() const{
  return g_odr;
}

void LSM6DS33::set_xl_odr(uint8_t odr){
  xl_odr= odr;
}
uint8_t LSM6DS33::get_xl_odr() const{
  return xl_odr;
}

void LSM6DS33::set_g_fs(uint8_t fs){
  g_fs= fs;
}
uint8_t LSM6DS33::get_g_fs() const{
  return g_fs;
}

void LSM6DS33::set_bdu(bool bdu){
  this->bdu= bdu;
}
bool LSM6DS33::get_bdu() const{
  return bdu;
}

bool LSM6DS33::get_g_burst() const{
  return g_burst;
}

void LSM6DS33::set_drdy_mask(bool drdy_mask){
  this->drdy_mask= drdy_mask;
}
bool LSM6DS33::get_drdy_mask() const{
  return drdy_mask;
}



uint8_t LSM6DS33::get_whoami() const{
  return whoami;
}

bool LSM6DS33::get_booting() const{
  return STATUS_REG & 1<<3;
}
bool LSM6DS33::get_tda() const{
  return STATUS_REG & 1<<2;
}
bool LSM6DS33::get_gda() const{
  return STATUS_REG & 1<<1;
}
bool LSM6DS33::get_xlda() const{
  return STATUS_REG & 1<<0;
}

int16_t LSM6DS33::temp() const{
  return tempout;
}

int16_t LSM6DS33::g_x() const{
  return gout[0];
}
int16_t LSM6DS33::g_y() const{
  return gout[1];
}
int16_t LSM6DS33::g_z() const{
  return gout[2];
}

int16_t LSM6DS33::xl_x() const{
  return xlout[0];
}
int16_t LSM6DS33::xl_y() const{
  return xlout[1];
}
int16_t LSM6DS33::xl_z() const{
  return xlout[2];
}

//
