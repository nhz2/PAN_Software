/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM6DSM is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LSM6DSM_h
#define LSM6DSM_h

#include "Arduino.h"
#include "I2CDevice.hpp"
#include <i2c_t3_pan/i2c_t3_pan.h>

/* LSM6DSM registers
  http://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
*/

namespace Devices {
class LSM6DSM : public I2CDevice {
  public:
    enum REGISTERS {
      FUNC_CFG_ACCESS           = 0x01,
      SENSOR_SYNC_TIME_FRAME    = 0x04,
      SENSOR_SYNC_RES_RATIO     = 0x05,
      FIFO_CTRL1                = 0x06,
      FIFO_CTRL2                = 0x07,
      FIFO_CTRL3                = 0x08,
      FIFO_CTRL4                = 0x09,
      FIFO_CTRL5                = 0x0A,
      DRDY_PULSE_CFG            = 0x0B,
      INT1_CTRL                 = 0x0D,
      INT2_CTRL                 = 0x0E,
      WHO_AM_I                  = 0x0F,  // should be = 0x6A
      CTRL1_XL                  = 0x10,
      CTRL2_G                   = 0x11,
      CTRL3_C                   = 0x12,
      CTRL4_C                   = 0x13,
      CTRL5_C                   = 0x14,
      CTRL6_C                   = 0x15,
      CTRL7_G                   = 0x16,
      CTRL8_XL                  = 0x17,
      CTRL9_XL                  = 0x18,
      CTRL10_C                  = 0x19,
      MASTER_CONFIG             = 0x1A,
      WAKE_UP_SRC               = 0x1B,
      TAP_SRC                   = 0x1C,
      D6D_SRC                   = 0x1D,
      STATUS_REG                = 0x1E,
      OUT_TEMP_L                = 0x20,
      OUT_TEMP_H                = 0x21,
      OUTX_L_G                  = 0x22,
      OUTX_H_G                  = 0x23,
      OUTY_L_G                  = 0x24,
      OUTY_H_G                  = 0x25,
      OUTZ_L_G                  = 0x26,
      OUTZ_H_G                  = 0x27,
      OUTX_L_XL                 = 0x28,
      OUTX_H_XL                 = 0x29,
      OUTY_L_XL                 = 0x2A,
      OUTY_H_XL                 = 0x2B,
      OUTZ_L_XL                 = 0x2C,
      OUTZ_H_XL                 = 0x2D,
      SENSORHUB1_REG            = 0x2E,
      SENSORHUB2_REG            = 0x2F,
      SENSORHUB3_REG            = 0x30,
      SENSORHUB4_REG            = 0x31,
      SENSORHUB5_REG            = 0x32,
      SENSORHUB6_REG            = 0x33,
      SENSORHUB7_REG            = 0x34,
      SENSORHUB8_REG            = 0x35,
      SENSORHUB9_REG            = 0x36,
      SENSORHUB10_REG           = 0x37,
      SENSORHUB11_REG           = 0x38,
      SENSORHUB12_REG           = 0x39,
      FIFO_STATUS1              = 0x3A,
      FIFO_STATUS2              = 0x3B,
      FIFO_STATUS3              = 0x3C,
      FIFO_STATUS4              = 0x3D,
      FIFO_DATA_OUT_L           = 0x3E,
      FIFO_DATA_OUT_H           = 0x3F,
      TIMESTAMP0_REG            = 0x40,
      TIMESTAMP1_REG            = 0x41,
      TIMESTAMP2_REG            = 0x42,
      STEP_TIMESTAMP_L          = 0x49,
      STEP_TIMESTAMP_H          = 0x4A,
      STEP_COUNTER_L            = 0x4B,
      STEP_COUNTER_H            = 0x4C,
      SENSORHUB13_REG           = 0x4D,
      SENSORHUB14_REG           = 0x4E,
      SENSORHUB15_REG           = 0x4F,
      SENSORHUB16_REG           = 0x50,
      SENSORHUB17_REG           = 0x51,
      SENSORHUB18_REG           = 0x52,
      FUNC_SRC1                 = 0x53,
      FUNC_SRC2                 = 0x54,
      WRIST_TILT_IA             = 0x55,
      TAP_CFG                   = 0x58,
      TAP_THS_6D                = 0x59,
      INT_DUR2                  = 0x5A,
      WAKE_UP_THS               = 0x5B,
      WAKE_UP_DUR               = 0x5C,
      FREE_FALL                 = 0x5D,
      MD1_CFG                   = 0x5E,
      MD2_CFG                   = 0x5F,
      MASTER_MODE_CODE          = 0x60,
      SENS_SYNC_SPI_ERROR_CODE  = 0x61,
      OUT_MAG_RAW_X_L           = 0x66,
      OUT_MAG_RAW_X_H           = 0x67,
      OUT_MAG_RAW_Y_L           = 0x68,
      OUT_MAG_RAW_Y_H           = 0x69,
      OUT_MAG_RAW_Z_L           = 0x6A,
      OUT_MAG_RAW_Z_H           = 0x6B,
      INT_OIS                   = 0x6F,
      CTRL1_OIS                 = 0x70,
      CTRL2_OIS                 = 0x71,
      CTRL3_OIS                 = 0x72,
      X_OFS_USR                 = 0x73,
      Y_OFS_USR                 = 0x74,
      Z_OFS_USR                 = 0x75
    };

    static constexpr uint8_t ADDRESS = 0x6A;   // Address of LSM6DSM accel/gyro when ADO = 0

    enum ACCEL_SCALE {
      AFS_2G  = 0x00,
      AFS_4G  = 0x02,
      AFS_8G  = 0x03,
      AFS_16G = 0x01
    };

    enum GYRO_SCALE {
      GFS_245DPS  = 0x00,
      GFS_500DPS  = 0x01,
      GFS_1000DPS = 0x02,
      GFS_2000DPS = 0x03
    };

    enum A_ODR_RATES {
      AODR_12_5Hz  = 0x01,  // same for accel and gyro in normal mode
      AODR_26Hz    = 0x02,
      AODR_52Hz    = 0x03,
      AODR_104Hz   = 0x04,
      AODR_208Hz   = 0x05,
      AODR_416Hz   = 0x06,
      AODR_833Hz   = 0x07,
      AODR_1660Hz  = 0x08,
      AODR_3330Hz  = 0x09,
      AODR_6660Hz  = 0x0A,
    };

    enum G_ODR_RATES {
      GODR_12_5Hz  = 0x01,  
      GODR_26Hz    = 0x02,
      GODR_52Hz    = 0x03,
      GODR_104Hz   = 0x04,
      GODR_208Hz   = 0x05,
      GODR_416Hz   = 0x06,
      GODR_833Hz   = 0x07,
      GODR_1660Hz  = 0x08,
      GODR_3330Hz  = 0x09,
      GODR_6660Hz  = 0x0A
    };

    LSM6DSM(i2c_t3 &i2c, uint8_t int_pin1, uint8_t int_pin2, uint8_t i2c_addr);

    bool setup() override;
    void reset() override;
    bool i2c_ping() override;
    void disable() override;
    void single_comp_test() override;

    float get_a_res(ACCEL_SCALE a_scale);
    float get_g_res(GYRO_SCALE g_scale);
    uint8_t get_chip_id();
    
    void init(ACCEL_SCALE a_scale, GYRO_SCALE g_scale, A_ODR_RATES A_ODR, G_ODR_RATES G_ODR);
    void offset_bias(float * dest1, float * dest2);

    void read_data(float* destination);
  private:
    uint8_t _int_pin1;
    uint8_t _int_pin2;
    float _a_res, _g_res;
};
}

#endif