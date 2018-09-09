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
    //// Ensure # and type of openings = # and type of closings
    // Ensure # of openings = # of closings
    if (schedule.openings.size() != schedule.closings.size()) return false;
    // Ensure each valve that's opened is also closed.
    auto openings_it = schedule.openings.begin();
    while (openings_it != schedule.openings.end()) {
        bool closing_found = false;

        auto closings_it = schedule.closings.begin();
        do {
            if (closings_it->second == openings_it->second) {
                schedule.closings.erase(closings_it);
                closing_found = true;
            }
            closings_it++;
        } while (closings_it != schedule.closings.end());
        
        if (!closing_found) return false;
        openings_it++;
    }
    //// Ensure last openings and closings are not beyond the micros() range.
    uint32_t last_opening = schedule.openings.end()->second;
    uint32_t last_closing = schedule.openings.end()->second;
    if (last_opening * 1000 > ULONG_MAX) return false;
    if (last_closing * 1000 > ULONG_MAX) return false;

    return true;
}