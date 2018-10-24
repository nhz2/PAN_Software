#include <cmath>
#include "MMC5883MA.h"

using namespace Devices;

MMC5883MA::MMC5883MA(i2c_t3 &i2c_wire, uint8_t i2c_addr) : I2CDevice(i2c_wire, i2c_addr, 0) {
    id = i2c_read_from_subaddr(REGISTER_ADDRESSES::MMC5883MA_PRODUCT_ID);
    i2c_write_to_subaddr(REGISTER_ADDRESSES::MMC5883MA_INTERNAL_CONTROL_2,0x03);
    i2c_write_to_subaddr(REGISTER_ADDRESSES::MMC5883MA_INTERNAL_CONTROL_0,0x01);
}

void MMC5883MA::get_mag(magnetic_field_t* mag_field){
    uint8_t out[6];
    i2c_read_from_subaddr(REGISTER_ADDRESSES::MMC5883MA_OUT,out,6);
    mag_field->x = this->out_to_mag(out[0],out[1]);
    mag_field->y = this->out_to_mag(out[2],out[3]);
    mag_field->z = this->out_to_mag(out[4],out[5]);
    this->xyz_to_dif(mag_field);
}

inline float MMC5883MA::out_to_mag(uint8_t LSB, uint8_t MSB) {
    return (float)(MSB << 8 | LSB) * MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION 
        - (float)MMC5883MA_DYNAMIC_RANGE / 2;}

void MMC5883MA::xyz_to_dif(magnetic_field_t* mag_field){
	mag_field->d = atan2(mag_field->y,mag_field->x);
	mag_field->i = atan(sqrt(pow(mag_field->x,2) + pow(mag_field->y,2)) / mag_field->z);
	mag_field->f = sqrt(pow(mag_field->x,2) + pow(mag_field->y,2) + pow(mag_field->z,2));
}