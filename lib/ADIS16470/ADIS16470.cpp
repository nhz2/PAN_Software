////////////////////////////////////////////////////////////////////////////////////////////////////////
//  November 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16470.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16470 IMU with a 
//  PJRC 32-Bit Teensy 3.2 Development Board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1646X family of devices 
//  with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ADIS16470.h"

using namespace Devices;

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16470::ADIS16470(uint8_t CS, uint8_t DR, uint8_t RST) {
    _CS = CS;
    _DR = DR;
    _RST = RST;
    // Initialize SPI
    SPI.begin();
    // Configure SPI controller
    config_SPI();
    // Set default pin states
    pinMode(_CS, OUTPUT); // Set CS pin to be an output
    pinMode(_DR, INPUT); // Set DR pin to be an input
    pinMode(_RST, OUTPUT); // Set RST pin to be an output
    digitalWrite(_CS, HIGH); // Initialize CS pin to be high
    digitalWrite(_RST, HIGH); // Initialize RST pin to be high
}

bool ADIS16470::setup() { return config_SPI(); }
void ADIS16470::reset() { reset_DUT(5); }
void ADIS16470::disable() { /** TODO **/ }
bool ADIS16470::is_functional() {
    return (reg_read(REGISTER_ADDRESSES::DIAG_STAT) == 0);
}
void ADIS16470::single_comp_test() { /** TODO **/ }

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16470::~ADIS16470() {
    // Close SPI bus
    SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting _RST pin low for delay (in ms).
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
bool ADIS16470::reset_DUT(uint8_t ms) {
    digitalWrite(_RST, LOW);
    delay(100);
    digitalWrite(_RST, HIGH);
    delay(ms);
    return true;
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
bool ADIS16470::config_SPI() {
    SPISettings IMUSettings(1000000, MSBFIRST, SPI_MODE3);
    SPI.beginTransaction(IMUSettings);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
// Returns an (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16470::reg_read(uint8_t reg_addr) {
    //Read registers using SPI
  
    // Write register address to be read
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(reg_addr); // Write address over SPI bus
    SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
    digitalWrite(_CS, HIGH); // Set CS high to disable device

    delayMicroseconds(_stall); // Delay to not violate read rate 

    // Read data from requested register
    digitalWrite(_CS, LOW); // Set CS low to enable device
    uint8_t _msb_data = SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
    uint8_t _lsb_data = SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
    digitalWrite(_CS, HIGH); // Set CS high to disable device

    delayMicroseconds(_stall); // Delay to not violate read rate 
  
    int16_t _data_out = (_msb_data << 8) | (_lsb_data & 0xFF); // Concatenate upper and lower bytes
    // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

    return(_data_out);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
bool ADIS16470::reg_write(uint8_t reg_addr, int16_t reg_data) {

    // Write register address and data
    uint16_t addr = (((reg_addr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
    uint16_t low_word = (addr | (reg_data & 0xFF)); // OR Register address (A) with data(D) (AADD)
    uint16_t high_word = ((addr | 0x100) | ((reg_data >> 8) & 0xFF)); // OR Register address with data and increment address

    // Split words into chars
    uint8_t high_byte_high_word = (high_word >> 8);
    uint8_t low_byte_high_word = (high_word & 0xFF);
    uint8_t high_byte_low_word = (low_word >> 8);
    uint8_t low_byte_low_word = (low_word & 0xFF);

    // Write high_word to SPI bus
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(high_byte_low_word); // Write high byte from low word to SPI bus
    SPI.transfer(low_byte_low_word); // Write low byte from low word to SPI bus
    digitalWrite(_CS, HIGH); // Set CS high to disable device

    delayMicroseconds(_stall);; // Delay to not violate read rate 

    // Write low_word to SPI bus
    digitalWrite(_CS, LOW); // Set CS low to enable device
    SPI.transfer(high_byte_high_word); // Write high byte from high word to SPI bus
    SPI.transfer(low_byte_high_word); // Write low byte from high word to SPI bus
    digitalWrite(_CS, HIGH); // Set CS high to disable device

    delayMicroseconds(_stall);; // Delay to not violate read rate 

    return true;
}

////////////////////////////////////////////////////////////////////////////
// Intiates a burst read from the sensor.
// Returns a pointer to an array of sensor data. 
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////
uint8_t *ADIS16470::byte_burst(void) {
    static uint8_t burst_data[20];

    // Trigger Burst Read
    digitalWrite(_CS, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0x00);

    // Read Burst Data
    burst_data[0] = SPI.transfer(0x00); //DIAG_STAT
    burst_data[1] = SPI.transfer(0x00);
    burst_data[2] = SPI.transfer(0x00); //XGYRO_OUT
    burst_data[3] = SPI.transfer(0x00);
    burst_data[4] = SPI.transfer(0x00); //YGYRO_OUT
    burst_data[5] = SPI.transfer(0x00);
    burst_data[6] = SPI.transfer(0x00); //ZGYRO_OUT
    burst_data[7] = SPI.transfer(0x00);
    burst_data[8] = SPI.transfer(0x00); //XACCEL_OUT
    burst_data[9] = SPI.transfer(0x00);
    burst_data[10] = SPI.transfer(0x00); //YACCEL_OUT
    burst_data[11] = SPI.transfer(0x00);
    burst_data[12] = SPI.transfer(0x00); //ZACCEL_OUT
    burst_data[13] = SPI.transfer(0x00);
    burst_data[14] = SPI.transfer(0x00); //TEMP_OUT
    burst_data[15] = SPI.transfer(0x00);
    burst_data[16] = SPI.transfer(0x00); //TIME_STMP
    burst_data[17] = SPI.transfer(0x00);
    burst_data[18] = SPI.transfer(0x00); //CHECKSUM
    burst_data[19] = SPI.transfer(0x00);
    digitalWrite(_CS, HIGH);

    return burst_data;
}

////////////////////////////////////////////////////////////////////////////
// Intiates a burst read from the sensor.
// Returns a pointer to an array of sensor data. 
////////////////////////////////////////////////////////////////////////////
// No inputs required.
////////////////////////////////////////////////////////////////////////////
uint16_t *ADIS16470::word_burst(void) {
    static uint16_t burst_words[10];

    // Trigger Burst Read
    digitalWrite(_CS, LOW);
    SPI.transfer(0x68);
    SPI.transfer(0x00);

    // Read Burst Data
    burst_words[0] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //DIAG_STAT
    burst_words[1] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //XGYRO
    burst_words[2] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //YGYRO
    burst_words[3] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //ZGYRO
    burst_words[4] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //XACCEL
    burst_words[5] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //YACCEL
    burst_words[6] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //ZACCEL
    burst_words[7] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //TEMP_OUT
    burst_words[8] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //TIME_STMP
    burst_words[9] = ((SPI.transfer(0x00) << 8) | (SPI.transfer(0x00) & 0xFF)); //CHECKSUM

    digitalWrite(_CS, HIGH);

    return burst_words;
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////
// *burst_array - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////
int16_t ADIS16470::checksum(uint16_t * burst_array) {
    int16_t s = 0;
    for (int i = 0; i < 9; i++) // Checksum value is not part of the sum!!
    {
        s += (burst_array[i] & 0xFF); // Count lower byte
        s += ((burst_array[i] >> 8) & 0xFF); // Count upper byte
    }

    return s;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function
// Returns (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16470::accel_scale(int16_t sensor_data) {
    float final_data = sensor_data * 0.00125; // Multiply by accel sensitivity (0.00125g/LSB)
    return final_data;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function 
// Returns (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16470::gyro_scale(int16_t sensor_data) {
    float final_data = sensor_data * 0.1; // Multiply by gyro sensitivity (0.1 deg/LSB)
    return final_data;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function 
// Returns (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16470::temp_scale(int16_t sensor_data) {
    float final_data = (sensor_data * 0.1); // Multiply by temperature scale (0.1 deg C/LSB)
    return final_data;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated angle data output from the regRead() function 
// Returns (float) signed/scaled delta angle in degrees
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16470::delta_angle_scale(int16_t sensor_data) {
    float final_data = sensor_data * 0.061; // Multiply by delta angle scale (0.061 degrees/LSB)
    return final_data;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts integrated velocity data output from the regRead() function 
// Returns (float) signed/scaled delta velocity in m/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16470::delta_velocity_scale(int16_t sensor_data) {
    float final_data = sensor_data * 0.01221; // Multiply by velocity scale (0.01221 m/sec/LSB)
    return final_data;
}