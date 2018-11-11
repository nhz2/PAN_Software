#include <cstdint>
#include <MMC5883MA/MMC5883MA.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>

using namespace Devices;

MMC5883MA imu(Wire, MMC5883MA::ADDR);

void setup() {
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
    Serial.begin(115200);
    imu.setup();
}

#define PACKET_SIZE 16
void loop() {
    uint8_t packet[PACKET_SIZE];
    MMC5883MA::magnetic_field_t magfield;

    imu.get_mag(&magfield);
    uint32_t sample_time = micros();

    Serial.printf("%d,%f,%f,%f\n", sample_time, magfield.x, magfield.y, magfield.z);
}