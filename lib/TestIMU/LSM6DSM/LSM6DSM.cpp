/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM6DSM is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include <stdio.h>
#include "LSM6DSM.h"

using namespace Devices;

LSM6DSM::LSM6DSM(i2c_t3 &i2c, uint8_t int_pin1, uint8_t int_pin2, uint8_t i2c_addr) 
  : I2CDevice(i2c, i2c_addr, 0), _int_pin1(int_pin1), _int_pin2(int_pin2)  { }

bool LSM6DSM::setup() {
    bool setup_succeeded = I2CDevice::setup();
    if (!setup_succeeded) return false;
    pinMode(_int_pin1, INPUT);
    pinMode(_int_pin2, INPUT);
    return true;
}

void LSM6DSM::disable() { I2CDevice::disable(); } // TODO

bool LSM6DSM::i2c_ping() {
  return true;
}

uint8_t LSM6DSM::get_chip_id() {
  uint8_t c = i2c_read_from_subaddr(REGISTERS::WHO_AM_I);
  return c;
}

float LSM6DSM::get_a_res(LSM6DSM::ACCEL_SCALE a_scale) {
  switch (a_scale) {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      _a_res = 2.0f/32768.0f;
      return _a_res;
      break;
    case AFS_4G:
      _a_res = 4.0f/32768.0f;
      return _a_res;
      break;
    case AFS_8G:
      _a_res = 8.0f/32768.0f;
      return _a_res;
      break;
    case AFS_16G:
      _a_res = 16.0f/32768.0f;
      return _a_res;
      break;
    default: // 2G
      _a_res = 2.0f/32768.0f;
      return _a_res;
  }
}

float LSM6DSM::get_g_res(LSM6DSM::GYRO_SCALE g_scale) {
  switch (g_scale) {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_245DPS:
      _g_res = 245.0f/32768.0f;
      return _g_res;
      break;
    case GFS_500DPS:
      _g_res = 500.0f/32768.0f;
      return _g_res;
      break;
    case GFS_1000DPS:
      _g_res = 1000.0f/32768.0f;
      return _g_res;
      break;
    case GFS_2000DPS:
      _g_res = 2000.0f/32768.0f;
      return _g_res;
      break;
    default: //245 dps
      _g_res = 245.0f/32768.0f;
      return _g_res;
  }
}

void LSM6DSM::reset() {
  // reset device
  uint8_t temp = i2c_read_from_subaddr(REGISTERS::CTRL3_C);
  i2c_write_to_subaddr(REGISTERS::CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSM
  delay(100); // Wait for all registers to reset 
}

void LSM6DSM::init(LSM6DSM::ACCEL_SCALE a_scale, LSM6DSM::GYRO_SCALE g_scale, LSM6DSM::A_ODR_RATES A_ODR, LSM6DSM::G_ODR_RATES G_ODR) {
  i2c_write_to_subaddr(REGISTERS::CTRL1_XL, A_ODR << 4 | a_scale << 2);
  
  i2c_write_to_subaddr(REGISTERS::CTRL2_G, G_ODR << 4 | g_scale << 2);
 
  uint8_t temp = i2c_read_from_subaddr(REGISTERS::CTRL3_C);
  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  i2c_write_to_subaddr(REGISTERS::CTRL3_C, temp | 0x40 | 0x04); 
  // by default, interrupts active HIGH, push pull, little endian data 
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

  // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
  i2c_write_to_subaddr(REGISTERS::CTRL8_XL, 0x80 | 0x40 | 0x08 );

  // interrupt handling
  i2c_write_to_subaddr(REGISTERS::DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
  i2c_write_to_subaddr(REGISTERS::INT1_CTRL, 0x40);      // enable significant motion interrupts on INT1
  i2c_write_to_subaddr(REGISTERS::INT2_CTRL, 0x03);      // enable accel/gyro data ready interrupts on INT2  
}

void LSM6DSM::read_data(float* destination) {
  uint8_t raw_data[14];  // x/y/z accel register data stored here
  i2c_read_from_subaddr(REGISTERS::OUT_TEMP_L, raw_data, 14);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0] ;  // Turn the MSB and LSB into a signed 16-bit value // Temperature
  destination[1] = (((int16_t)raw_data[3] << 8) | raw_data[2]) * _g_res; // Gyro X  
  destination[2] = (((int16_t)raw_data[5] << 8) | raw_data[4]) * _g_res; // Gyro Y 
  destination[3] = (((int16_t)raw_data[7] << 8) | raw_data[6]) * _g_res; // Gyro Z 
  destination[4] = (((int16_t)raw_data[9] << 8) | raw_data[8]) * _a_res; // Accel X  
  destination[5] = (((int16_t)raw_data[11] << 8) | raw_data[10]) * _a_res; // Accel Y  
  destination[6] = (((int16_t)raw_data[13] << 8) | raw_data[12]) * _a_res; // Accel Z 
}