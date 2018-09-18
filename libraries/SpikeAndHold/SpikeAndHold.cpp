#include <stdlib>
#include <stdint>
#include <Device.hpp>
#include <SpikeAndHold.hpp>

using namespace Devices;

SpikeAndHold::SpikeAndHold() {
    uint8_t PIN_VALUES[6] = {3, 4, 5, 6, 9, 10};
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        valve_pins[i] = PIN_VALUES[i];
    }
}

SpikeAndHold::SpikeAndHold(const uint8_t[SpikeAndHold::NUM_VALVES] pins) {
    for (uint8_t i = 0; i < NUM_VALVES; i++) {
        valve_pins[i] = pins[i];
    }
}

bool SpikeAndHold::fire_thrusters(const SpikeAndHold::FiringSchedule &schedule) {
    if (!_schedule_is_correct(schedule)) return false;
    bool done = false;
    uint32_t start_time = micros();

    while(!done) {
        bool done_openings = false;
        bool done_closings = false;
        uint32_t it_time = micros() - start_time;

        if (done_openings && done_closings) {
            done = true;
        }
        else if (!done_openings) {
            // Open all valves that should be opened at this time.
            if (schedule.openings.size() != 0) {
                auto next_opening = schedule.openings.begin();
                uint32_t next_opening_time = next_opening->first;
                uint8_t next_opening_valve = valve_pins[next_openings->second];
                if (next_opening_time * 1000 - it_time >= 0) {
                    digitalWrite(next_opening_valve, HIGH);
                }
            }
            else {
                done_openings = true;
            }
        }
        else if (!done_closings) {
            // Close all valves that should be closed at this time.
            if (schedule.closings.size() != 0) {
                auto next_closing = schedule.closings.begin();
                uint32_t next_closing_time = next_closing->first;
                uint8_t next_closing_valve = valve_pins[next_closing->second];
                if (next_closing_time * 1000 - it_time >= 0) {
                    digitalWrite(next_closing_valve, LOW);
                }
            }
            else {
                done_closings = true;
            }
        }
    }    
    return true;
}

bool SpikeAndHold::_schedule_is_correct(const SpikeAndHold::FiringSchedule &schedule) {
    uint32_t **o = schedule.openings;
    uint32_t **c = schedule.closings;
    uint8_t o_size = sizeof(o) / sizeof(uint32_t);
    uint8_t c_size = sizeof(c) / sizeof(uint32_t);
    if (o_size != c_size) return false;

    //// Ensure both lists are ordered, and that all valves that would be opened are valid
    // Ensure openings are ordered and are valid valves
    for(uint8_t i = 1; i < o_size; i++) {
        if (o[i][0] <= o[i - 1][0]) return false;
        if (o[i][1] >= NUM_VALVES) return false;
    }
    // Ensure closings are ordered and are valid valves
    for(uint8_t i = 1; i < c_size; i++) {
        if (c[i][0] <= c[i - 1][0]) return false;
        if (c[i][1] >= NUM_VALVES) return false;
    }

    //// Ensure # of openings per valve = # of closings per valve
    uint8_t num_openings[NUM_VALVES]; // Number of times valve i is opened
    uint8_t num_closings[NUM_VALVES]; // Number of times valve i is opened
    for(uint8_t i = 0; i < NUM_VALVES; i++) { num_openings[i] = 0; num_closings[i] = 0; }
    for(uint8_t i = 0; i < o_size; i++) { num_openings[o[i][1]]++; }
    for(uint8_t i = 0; i < c_size; i++) { num_openings[c[i][1]]++; }
    for(uint8_t i = 0; i < NUM_VALVES; i++) { if (num_openings[i] != num_closings[i]) return false; }

    //// Ensure last openings and closings are not beyond the micros() range.
    uint32_t last_opening = o[o_size - 1][1];
    uint32_t last_closing = c[c_size - 1][1];
    if (last_opening * 1000 > ULONG_MAX) return false;
    if (last_closing * 1000 > ULONG_MAX) return false;

    return true;
}