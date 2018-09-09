#include <SpikeAndHold.hpp>

using namespace Devices;

SpikeAndHold sh;

*SpikeAndHold::FiringSchedule create_single_firing(uint8_t pin, uint32_t duration) {
    SpikeAndHold::FiringSchedule *schedule = new SpikeAndHold::FiringSchedule();
    schedule->openings.insert(schedule->openings.begin(), 0);
    schedule->closings.insert(schedule->closings.begin(), duration);
    return schedule;
}

void setup() {
}

void loop() {
    uint8_t PIN = 0;
    uint8_t DURATION = 50;
    SpikeAndHold::FiringSchedule fs = *create_single_firing(PIN, DURATION);
    sh.fire_thrusters(fs);
}