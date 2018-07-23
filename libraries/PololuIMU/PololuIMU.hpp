//
// libraries/PololuIMU/PololuIMU.hpp
//
// Created by Kyle Krol (kpk63@cornell.edu) on 06/08/2018
// Gyro and Accelerometer drivers created by Nathan Zimmerberg (nhz2@cornell.edu) on 07/10/2018
//
// using register list from https://github.com/pololu/lsm6-arduino/blob/master/LSM6.h
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell University
//

#ifndef LIBRARIES_POLOLUIMU_POLOLUIMU_HPP
#define LIBRARIES_POLOLUIMU_POLOLUIMU_HPP

#include <I2CDevice.hpp>


/*! Possible slave addresses */
static const uint8_t LIS3MDL_ADDR_HIGH = 0b0011110;
static const uint8_t LIS3MDL_ADDR_LOW  = 0b0011100;

#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010
#define DS33_WHO_ID    0x69

/*! Magnetometer registers */
static const uint8_t LIS3MDL_REG_CTRL1  = 0x20;
static const uint8_t LIS3MDL_REG_CTRL2  = 0x21;
static const uint8_t LIS3MDL_REG_CTRL3  = 0x22;
static const uint8_t LIS3MDL_REG_CTRL4  = 0x23;
static const uint8_t LIS3MDL_REG_CTRL5  = 0x24;
static const uint8_t LIS3MDL_REG_OUT_XL = 0x28;
static const uint8_t LIS3MDL_REG_OUT_XH = 0x29;
static const uint8_t LIS3MDL_REG_OUT_YL = 0x2A;
static const uint8_t LIS3MDL_REG_OUT_YH = 0x2B;
static const uint8_t LIS3MDL_REG_OUT_ZL = 0x2C;
static const uint8_t LIS3MDL_REG_OUT_ZH = 0x2D;

/*! z-axis and xy-axis performance constants */
static const uint8_t LIS3MDL_PERF_LOW    = 0b0000;
static const uint8_t LIS3MDL_PERF_MEDIUM = 0b0100;
static const uint8_t LIS3MDL_PERF_HIGH   = 0b1000;
static const uint8_t LIS3MDL_PERF_ULTRA  = 0b1100;

/*! Possible sample frequencies */
static const uint8_t LIS3MDL_SF_0_625 = 0b00000;
static const uint8_t LIS3MDL_SF_1_25  = 0b00100;
static const uint8_t LIS3MDL_SF_2_5   = 0b01000;
static const uint8_t LIS3MDL_SF_5     = 0b01100;
static const uint8_t LIS3MDL_SF_10    = 0b10000;
static const uint8_t LIS3MDL_SF_20    = 0b10100;
static const uint8_t LIS3MDL_SF_40    = 0b11000;
static const uint8_t LIS3MDL_SF_80    = 0b11100;

/*! Possible magnetometer scaling values */
static const uint8_t LIS3MDL_SCALE_PM4G  = 0b0000000;
static const uint8_t LIS3MDL_SCALE_PM8G  = 0b0100000;
static const uint8_t LIS3MDL_SCALE_PM12G = 0b1000000;
static const uint8_t LIS3MDL_SCALE_PM16G = 0b1100000;

class LIS3MDL : public I2CDevice {

public:

  /*! Constructs a magnetometer on the specified wire and with the given
   *  address.
   */
  LIS3MDL(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  /*! Sets and returns the current sample frequency. The default sample
   * frequency is 0.625 Hz.
   */
  void set_sample_frequency(uint8_t sample_frequency);
  uint8_t get_sample_frequency() const;

  /*! Sets and returns the current xy component operating mode. The default mode
   * is low performace.
   */
  void set_xy_performance(uint8_t xy_performance);
  uint8_t get_xy_performance() const;

  /*! Sets and returns the current z component operating mode. The default mode
   * is low performace.
   */
  void set_z_performance(uint8_t z_performance);
  uint8_t get_z_performance() const;

  /*! Sets and returns the magnetometer scale value. The default full scale is
   * plus or minus 4 gauss.
   */
  void set_scale(uint8_t scale);
  uint8_t get_scale() const;

  /*! Returns magnetometer readings */
  int16_t x() const;
  int16_t y() const;
  int16_t z() const;

  /*! Reads the magnetometer and returns true if succesfful */
  bool read();

private:

  /*! Magnetic field readings */
  int16_t mag[3];

  /*! Configuration settings */
  uint8_t sample_frequency;
  uint8_t xy_performance;
  uint8_t z_performance;
  uint8_t scale;

  /*! Indicates whether control registers must be updated */
  bool should_reset;

};







/*!
 */
class LSM6DS33 {
  // register addresses from
  //https://github.com/pololu/lsm6-arduino/blob/master/LSM6.h
    enum regAddr
    {
      FUNC_CFG_ACCESS   = 0x01,

      FIFO_CTRL1        = 0x06,
      FIFO_CTRL2        = 0x07,
      FIFO_CTRL3        = 0x08,
      FIFO_CTRL4        = 0x09,
      FIFO_CTRL5        = 0x0A,
      ORIENT_CFG_G      = 0x0B,

      INT1_CTRL         = 0x0D,
      INT2_CTRL         = 0x0E,
      WHO_AM_I          = 0x0F,
      CTRL1_XL          = 0x10,
      CTRL2_G           = 0x11,
      CTRL3_C           = 0x12,
      CTRL4_C           = 0x13,
      CTRL5_C           = 0x14,
      CTRL6_C           = 0x15,
      CTRL7_G           = 0x16,
      CTRL8_XL          = 0x17,
      CTRL9_XL          = 0x18,
      CTRL10_C          = 0x19,

      WAKE_UP_SRC       = 0x1B,
      TAP_SRC           = 0x1C,
      D6D_SRC           = 0x1D,
      STATUS_REG        = 0x1E,

      OUT_TEMP_L        = 0x20,
      OUT_TEMP_H        = 0x21,
      OUTX_L_G          = 0x22,
      OUTX_H_G          = 0x23,
      OUTY_L_G          = 0x24,
      OUTY_H_G          = 0x25,
      OUTZ_L_G          = 0x26,
      OUTZ_H_G          = 0x27,
      OUTX_L_XL         = 0x28,
      OUTX_H_XL         = 0x29,
      OUTY_L_XL         = 0x2A,
      OUTY_H_XL         = 0x2B,
      OUTZ_L_XL         = 0x2C,
      OUTZ_H_XL         = 0x2D,

      FIFO_STATUS1      = 0x3A,
      FIFO_STATUS2      = 0x3B,
      FIFO_STATUS3      = 0x3C,
      FIFO_STATUS4      = 0x3D,
      FIFO_DATA_OUT_L   = 0x3E,
      FIFO_DATA_OUT_H   = 0x3F,
      TIMESTAMP0_REG    = 0x40,
      TIMESTAMP1_REG    = 0x41,
      TIMESTAMP2_REG    = 0x42,

      STEP_TIMESTAMP_L  = 0x49,
      STEP_TIMESTAMP_H  = 0x4A,
      STEP_COUNTER_L    = 0x4B,
      STEP_COUNTER_H    = 0x4C,

      FUNC_SRC          = 0x53,

      TAP_CFG           = 0x58,
      TAP_THS_6D        = 0x59,
      INT_DUR2          = 0x5A,
      WAKE_UP_THS       = 0x5B,
      WAKE_UP_DUR       = 0x5C,
      FREE_FALL         = 0x5D,
      MD1_CFG           = 0x5E,
      MD2_CFG           = 0x5F,
    };


public:
  /*! Constructs a LSM6 on the specified wire and with the given
   *  address and with the given odrs.
   default states:
      fullscale of gyro is +-2.182rad/s
      sensors are powered down
      drdy_mask is on
      bdu is on
   inputs:
   *   gyroscope output data rate = 6660Hz*2^(-10+xl_odr)
   *    (inclusive range from 1 to 8) cooresponding to 12.5 to 1660Hz
   *   accelerometer output data rate = 6660Hz*2^(-10+xl_odr)
   *    (inclusive range from 1 to 10) cooresponding to 12.5 to 6660Hz
   */
  LSM6DS33(i2c_t3 &i2c_wire, uint8_t i2c_addr, uint8_t g_odr, uint8_t xl_odr);

  /*! do the self test, takes about 10s, prints to serial readings to serial
   * 5 readings with self test off and 5 readings with self test on
   * order is + gyro, - gyro, + xl, - xl
   */
  void self_test();

  /*if sent i2c message to start up g and xl in high performance mode with set cfg
  wait about 20ms after booting before calling this function
  wait about 80ms after calling this function before reading
  see section 4.1 https://www.st.com/content/ccc/resource/technical/document/application_note/9c/d9/07/d0/d4/a9/45/00/DM00175930.pdf/files/DM00175930.pdf/jcr:content/translations/en.DM00175930.pdf
   */
  bool power_up();

  /*if sent i2c message to power down both g and xl.
  Must start up again before reading.
  wait 1 micro second before sending other messages
  Doesn't effect config settings*/
  bool power_down();

  /*if sent i2c message to reboot,
    device powered down after reboot
    from application note 5.7:
      After power-up, the trimming parameters can be re-loaded...
      No toggle of the device power lines is required and
      the content of the device control registers is not modified,
      so the device operating mode doesn’t change after boot...
      In order to avoid conflicts,
      the reboot and the sw reset must not be executed at the same time
    wait 20ms before sending other i2c messages*/
  bool reboot();

  /*if sent i2c message to reset software back to default
    In order to avoid conflicts,
    the reboot and the sw reset must not be executed at the same time
    wait 50 micro seconds before sending other messages*/
  bool reset();



  /*return the power state of the g and xl,
      true is both on in high per, false is both off*/
  bool get_power_state() const;

  // bool setup_fifo();
  //
  // bool read_fifo();
  //
  // bool setup_burst();
  //
  // bool burst_read();

  /*

  /*! gyroscope output data rate = 6660Hz*2^(-10+xl_odr)
  * (inclusive range from 1 to 8) cooresponding to 12.5 to 1660Hz
  */
  void set_g_odr(uint8_t odr);
  uint8_t get_g_odr() const;

  /*! accelerometer output data rate = 6660Hz*2^(-10+xl_odr)
  * (inclusive range from 1 to 10) cooresponding to 12.5 to 6660Hz
  */
  void set_xl_odr(uint8_t odr);
  uint8_t get_xl_odr() const;
  /*! gyroscope measurement range = +-2.182rad/s*2^(g_fs)
  * (inclusive range from 0 to 4)
  */
  void set_g_fs(uint8_t fs);
  uint8_t get_g_fs() const;

  /*! is block update feature on, highly recommended to be on*/
  void set_bdu(bool bdu);
  bool get_bdu() const;

  /*! is drdy_mask on*/
  void set_drdy_mask(bool drdy_mask);
  bool get_drdy_mask() const;

  /*if sent i2c message to update config based on drdy_mask, bdu, fs, and odr
    If odr is changed, 1 to 4 samples need to be discarded,
      see application note table 17 and 18*/
  bool update_cfg();

  /*if sent i2c message to read status and data,
  if not synced, bdu should be on*/
  bool read();

  /*!who am i register should be 69h after read if everything is working*/
  uint8_t get_whoami() const;
  /*! Boot running flag signal*/
  bool get_booting() const;
  /*! new temp data*/
  bool get_tda() const;
  /*! new gyro data*/
  bool get_gda() const;
  /*! new xl data*/
  bool get_xlda() const;

  /*! Returns temperature readings */
  int16_t temp() const;

  /*! Returns gyroscope readings */
  int16_t g_x() const;
  int16_t g_y() const;
  int16_t g_z() const;

  /*! Returns accelerometer readings */
  int16_t xl_x() const;
  int16_t xl_y() const;
  int16_t xl_z() const;






private:
  //from https://github.com/pololu/lsm6-arduino/blob/master/LSM6.h
  void writeReg(regAddr reg, uint8_t value);
  uint8_t readReg(regAddr reg);

  //Configuration settings

  /*! is accelerometer and gyroscope in power down mode*/
  bool powered_down;
  /*! gyroscope output data rate = 6660Hz*2^(-10+xl_odr)
  * (inclusive range from 1 to 8) cooresponding to 12.5 to 1660Hz
  */
  uint8_t g_odr;
  /*! accelerometer output data rate = 6660Hz*2^(-10+xl_odr)
  * (inclusive range from 1 to 10) cooresponding to 12.5 to 6660Hz
  */
  uint8_t xl_odr;
  /*! gyroscope measurement range = +-2.182rad/s*2^(g_fs)
  * (inclusive range from 0 to 4)
  */
  uint8_t g_fs;
  /*! is DRDY mask on*/
  bool drdy_mask;
  /*! is block update feature on*/
  bool bdu;
  /*! is fifo on, either bypass or continuous*/
  // bool fifo
  // /*! fifo gyroscope data decimation factors 0 means not in fifo
  // * Either fifo_g_dec or fifo_xl_dec must be 1, inclusive range 0 to 7
  // */
  // unit8_t fifo_g_dec;
  // /*! fifo accelerometer data decimation factors 0 means not in fifo
  // * Either fifo_g_dec or fifo_xl_dec must be 1, inclusive range 0 to 7
  // */
  // unit8_t fifo_xl_dec;
  // /*! fifo output data rate = 6660Hz*2^(-10+xl_odr)
  // * (inclusive range from 1 to 10) cooresponding to 12.5 to 6660Hz
  // */
  // uint8_t fifo_odr;
  // /*! fifo threshold*/
  // unit16_t fifo_threshold;
  // /*! does fifo stop on threshold*/
  // bool stop_on_fth;

  //Status states

  /*!who am i register should be 69h after read if everything is working*/
  uint8_t whoami;
  /*! status_reg
  bit 0 is new accelerometer data
  bit 1 is new gyroscope data
  bit 2 is new temperature data
  bit 3 is boot running flag signal*/
  uint8_t status;

  //Data states

  /*! temp output*/
  int16_t tempout;
  /*! gyroscope output*/
  int16_t gout[3];
  /*! accelerometer output*/
  int16_t xlout[3];


};

#endif
