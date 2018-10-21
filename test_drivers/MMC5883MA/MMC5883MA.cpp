#include <cmath>
#include "MMC5883MA.hpp"

MMC5883MA::MMC5883MA():i2c(I2C_SDA,I2C_SCL){
    this->getReg(MMC5883MA_PRODUCT_ID,&this->id,1);
    this->putReg(MMC5883MA_INTERNAL_CONTROL_2,0x03);
    this->putReg(MMC5883MA_INTERNAL_CONTROL_0,0x01);
}

void MMC5883MA::getMag(MAGFIELD* magField){
    char out[6];
    getReg(MMC5883MA_OUT,out,6);
    magField->x = this->out2Mag(out[0],out[1]);
    magField->y = this->out2Mag(out[2],out[3]);
    magField->z = this->out2Mag(out[4],out[5]);
    this->xyz2dif(magField);
}

void MMC5883MA::putReg(char addr,char data){
    this->dataWrite[0] = addr;
    this->dataWrite[1] = data;
    this->i2c.write(MMC5883MA_ADDR,dataWrite,2);
}

void MMC5883MA::getReg(char addr,char* data,int size){
    this->dataWrite[0] = addr;
    this->i2c.write(MMC5883MA_ADDR,dataWrite,1,1);
    this->i2c.read(MMC5883MA_ADDR,data,size);
}

inline float MMC5883MA::out2Mag(char LSB,char MSB) {
    return (float)(MSB << 8 | LSB) * MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION 
        - (float)MMC5883MA_DYNAMIC_RANGE / 2;}

void MMC5883MA::xyz2dif(MAGFIELD* magField){
	magField->d = atan2(magField->y,magField->x);
	magField->i = atan(sqrt(pow(magField->x,2) + pow(magField->y,2)) / magField->z);
	magField->f = sqrt(pow(magField->x,2) + pow(magField->y,2) + pow(magField->z,2));
}