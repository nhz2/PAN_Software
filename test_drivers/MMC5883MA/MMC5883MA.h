#ifndef __MMC5883MA_HPP__
#define __MMC5883MA_HPP__

#define MMC5883MA_OUT 0x00
#define MMC5883MA_XOUT 0x00
#define MMC5883MA_XOUT_LOW 0x00
#define MMC5883MA_XOUT_HIGH 0x01
#define MMC5883MA_YOUT 0x02
#define MMC5883MA_YOUT_LOW 0x02
#define MMC5883MA_YOUT_HIGH 0x03
#define MMC5883MA_ZOUT 0x04
#define MMC5883MA_ZOUT_LOW 0x04
#define MMC5883MA_ZOUT_HIGH 0x05
#define MMC5883MA_TEMPERATURE 0x06
#define MMC5883MA_STATUS 0x07
#define MMC5883MA_INTERNAL_CONTROL_0 0x08
#define MMC5883MA_INTERNAL_CONTROL_1 0x09
#define MMC5883MA_INTERNAL_CONTROL_2 0x0A
#define MMC5883MA_X_THRESHOLD 0x0B
#define MMC5883MA_Y_THRESHOLD 0x0C
#define MMC5883MA_Z_THRESHOLD 0x0D
#define MMC5883MA_PRODUCT_ID 0x2F
#define MMC5883MA_ADDR 0x60

#define MMC5883MA_DYNAMIC_RANGE 16
#define MMC5883MA_RESOLUTION 65536

class MAGFIELD{
    public:
        float x,y,z;
        float d,i,f;
};

class MMC5883MA{
    public:
        MMC5883MA();
        void getMag(MAGFIELD* magField);
        char id;
    private:
        I2C i2c;
        void putReg(char addr,char data);
        void getReg(char addr,char* data,int size);
        inline float out2Mag(char LSB,char MSB);
        void xyz2dif(MAGFIELD* magField);
        char dataWrite[2];
};

#endif