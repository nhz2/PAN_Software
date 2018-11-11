/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LIS2MDL is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LIS2MDL_h
#define LIS2MDL_h

#include "../Devices/I2CDevice.hpp"

namespace Devices {
class LIS2MDL : public I2CDevice {
  public:
    //Register map for LIS2MDL'
    // http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf
    enum REGISTERS {
      OFFSET_X_REG_H = 0x45,
      OFFSET_X_REG_L = 0x46,
      OFFSET_Y_REG_H = 0x47,
      OFFSET_Y_REG_L = 0x48,
      OFFSET_Z_REG_H = 0x49,
      OFFSET_Z_REG_L = 0x4A,
      WHO_AM_I = 0x4F,
      CFG_REG_A = 0x60,
      CFG_REG_B = 0x61,
      CFG_REG_C = 0x62,
      INT_CTRL_REG = 0x63,
      INT_SOURCE_REG = 0x64,
      INT_THS_L_REG = 0x65,
      INT_THS_H_REG = 0x66,
      STATUS_REG = 0x67,
      OUTX_L_REG = 0x68,
      OUTX_H_REG = 0x69,
      OUTY_L_REG = 0x6A,
      OUTY_H_REG = 0x6B,
      OUTZ_L_REG = 0x6C,
      OUTZ_H_REG = 0x6D,
      TEMP_OUT_L_REG = 0x6E,
      TEMP_OUT_H_REG = 0x6F
    };
    enum MODR {
      MODR_10Hz = 0x00,
      MODR_20Hz = 0x01,
      MODR_50Hz = 0x02,
      MODR_100Hz = 0x03
    };
    static constexpr uint8_t ADDRESS = 0x1E;

    LIS2MDL(i2c_t3 &i2c_wire, uint8_t int_pin = 0, uint8_t i2c_addr = ADDRESS);

    // Device functions
    bool setup() override;
    void reset() override;
    void single_comp_test() override;
    bool i2c_ping() override;

    uint8_t get_chip_id();
    void offset_bias(float * dest1, float * dest2);
    uint8_t status();
    void read_data(float* destination);
    void read_data(int16_t* destination);
    int16_t read_temperature();
  private:
    uint8_t _int_pin;
    float _m_res;
};
}

#endif