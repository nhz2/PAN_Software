#ifndef __MMC5883MA_HPP__
#define __MMC5883MA_HPP__

#include "../Devices/I2CDevice.hpp"

namespace Devices {
class MMC5883MA : public I2CDevice {
    public:
        struct magnetic_field_t {
            float x,y,z;
            float d,i,f;
        };

        enum REGISTER_ADDRESSES {
            MMC5883MA_OUT = 0x00,
            MMC5883MA_XOUT = 0x00,
            MMC5883MA_XOUT_LOW = 0x00,
            MMC5883MA_XOUT_HIGH = 0x01,
            MMC5883MA_YOUT = 0x02,
            MMC5883MA_YOUT_LOW = 0x02,
            MMC5883MA_YOUT_HIGH = 0x03,
            MMC5883MA_ZOUT = 0x04,
            MMC5883MA_ZOUT_LOW = 0x04,
            MMC5883MA_ZOUT_HIGH = 0x05,
            MMC5883MA_TEMPERATURE = 0x06,
            MMC5883MA_STATUS = 0x07,
            MMC5883MA_INTERNAL_CONTROL_0 = 0x08,
            MMC5883MA_INTERNAL_CONTROL_1 = 0x09,
            MMC5883MA_INTERNAL_CONTROL_2 = 0x0A,
            MMC5883MA_X_THRESHOLD = 0x0B,
            MMC5883MA_Y_THRESHOLD = 0x0C,
            MMC5883MA_Z_THRESHOLD = 0x0D,
            MMC5883MA_PRODUCT_ID = 0x2F,
        };

        static const uint8_t MMC5883MA_ADDR = 0x60;
        static const uint8_t MMC5883MA_DYNAMIC_RANGE = 16;
        static const uint32_t MMC5883MA_RESOLUTION = 65536;

        MMC5883MA(i2c_t3 &i2c_wire, uint8_t i2c_addr);
        void get_mag(magnetic_field_t* mag_field);
        char id;
    private:
        inline float out_to_mag(uint8_t LSB, uint8_t MSB);
        void xyz_to_dif(magnetic_field_t* mag_field);
        char data_write[2];
};
}

#endif