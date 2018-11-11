#include <cstdint>
#include <LIS2MDL/LIS2MDL.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>

using namespace Devices;

LIS2MDL imu(Wire);
PacketSerial pSerial;

void onPacketReceived(const uint8_t* buffer, size_t size) { /** Do nothing **/ }

void setup() {
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
    Serial.begin(115200);
    pSerial.setStream(&Serial);
    pSerial.setPacketHandler(&onPacketReceived);
    imu.setup();
}

#define PACKET_SIZE 18
void loop() {
    uint8_t packet[PACKET_SIZE];
    float data[3];

    imu.read_data(data);
    int16_t temp = imu.read_temperature();
    uint32_t sample_time = micros();
    packet[0] = sample_time & 0xFF000000;
    packet[1] = sample_time & 0x00FF0000;
    packet[2] = sample_time & 0x0000FF00;
    packet[3] = sample_time & 0x000000FF;
    packet[4] = ((uint32_t) data[0]) & 0xFF000000;
    packet[5] = ((uint32_t) data[0]) & 0x00FF0000;
    packet[6] = ((uint32_t) data[0]) & 0x0000FF00;
    packet[7] = ((uint32_t) data[0]) & 0x000000FF;
    packet[8] = ((uint32_t) data[1]) & 0xFF000000;
    packet[9] = ((uint32_t) data[1]) & 0x00FF0000;
    packet[10] = ((uint32_t) data[1]) & 0x0000FF00;
    packet[11] = ((uint32_t) data[1]) & 0x000000FF;
    packet[12] = ((uint32_t) data[2]) & 0xFF000000;
    packet[13] = ((uint32_t) data[2]) & 0x00FF0000;
    packet[14] = ((uint32_t) data[2]) & 0x0000FF00;
    packet[15] = ((uint32_t) data[2]) & 0x000000FF;
    packet[16] = temp & 0xFF00;
    packet[17] = temp & 0x00FF;

    pSerial.send(packet, PACKET_SIZE);
}