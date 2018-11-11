#include <cstdint>
#include <MMC5883MA/MMC5883MA.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>

using namespace Devices;

MMC5883MA imu(Wire, MMC5883MA::ADDR);
PacketSerial pSerial;

void onPacketReceived(const uint8_t* buffer, size_t size) { /** Do nothing **/ }

void setup() {
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
    Serial.begin(115200);
    pSerial.setStream(&Serial);
    pSerial.setPacketHandler(&onPacketReceived);
    imu.setup();
}

#define PACKET_SIZE 20
void loop() {
    uint8_t packet[PACKET_SIZE];
    float data[3];

    MMC5883MA::magnetic_field_t magfield;
    imu.get_mag(&magfield);
    float temp = imu.get_temp();
    uint32_t sample_time = micros();
    packet[0] = sample_time & 0xFF000000;
    packet[1] = sample_time & 0x00FF0000;
    packet[2] = sample_time & 0x0000FF00;
    packet[3] = sample_time & 0x000000FF;
    packet[4] = ((uint32_t) magfield.x) & 0xFF000000;
    packet[5] = ((uint32_t) magfield.x) & 0x00FF0000;
    packet[6] = ((uint32_t) magfield.x) & 0x0000FF00;
    packet[7] = ((uint32_t) magfield.x) & 0x000000FF;
    packet[8] = ((uint32_t) magfield.y) & 0xFF000000;
    packet[9] = ((uint32_t) magfield.y) & 0x00FF0000;
    packet[10] = ((uint32_t) magfield.y) & 0x0000FF00;
    packet[11] = ((uint32_t) magfield.y) & 0x000000FF;
    packet[12] = ((uint32_t) magfield.z) & 0xFF000000;
    packet[13] = ((uint32_t) magfield.z) & 0x00FF0000;
    packet[14] = ((uint32_t) magfield.z) & 0x0000FF00;
    packet[15] = ((uint32_t) magfield.z) & 0x000000FF;
    packet[16] = ((uint32_t) temp) & 0xFF000000;
    packet[17] = ((uint32_t) temp) & 0x00FF0000;
    packet[18] = ((uint32_t) temp) & 0x0000FF00;
    packet[19] = ((uint32_t) temp) & 0x000000FF;

    pSerial.send(packet, PACKET_SIZE);
}