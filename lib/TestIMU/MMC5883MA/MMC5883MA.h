#ifndef _MMC5883MA__HPP__
#define _MMC5883MA__HPP__

#include "../Devices/I2CDevice.hpp"

namespace Devices {
class MMC5883MA : public I2CDevice {
  public:
    struct magnetic_field_t {
        float x,y,z;
    };

    enum REGISTERS {
        OUT = 0x00,
        XOUT = 0x00,
        XOUT_LOW = 0x00,
        XOUT_HIGH = 0x01,
        YOUT = 0x02,
        YOUT_LOW = 0x02,
        YOUT_HIGH = 0x03,
        ZOUT = 0x04,
        ZOUT_LOW = 0x04,
        ZOUT_HIGH = 0x05,
        TEMPERATURE = 0x06,
        STATUS = 0x07,
        INTERNAL_CONTROL_0 = 0x08,
        INTERNAL_CONTROL_1 = 0x09,
        INTERNAL_CONTROL_2 = 0x0A,
        X_THRESHOLD = 0x0B,
        Y_THRESHOLD = 0x0C,
        Z_THRESHOLD = 0x0D,
        PRODUCT_ID = 0x2F,
    };

    static constexpr uint8_t ADDR = 0x30;

    MMC5883MA(i2c_t3 &i2c_wire, uint8_t i2c_addr);

    bool setup() override;
    bool i2c_ping() override;
    void reset() override;
    void disable() override;
    void single_comp_test() override;

    void get_mag(magnetic_field_t* mag_field);
    int16_t get_temp();
  private:
    float out_to_mag(uint8_t LSB, uint8_t MSB);
};
}

#endif