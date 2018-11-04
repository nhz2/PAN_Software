#include "Arduino.h"
#include <climits>
#include <vector>
#include <tuple>
#include <algorithm>
#include "../Devices/Device.hpp"
#include "SpikeAndHold.hpp"

using namespace Devices;

// Define static variables so that C++ doesn't complain;
uint8_t SpikeAndHold::DEFAULT_VALVE_PINS[6] = {3,4,5,6,27,28};
uint8_t SpikeAndHold::valve_pins[SpikeAndHold::NUM_VALVES];
SpikeAndHold::VALVE_STATE SpikeAndHold::valve_states[SpikeAndHold::NUM_VALVES];
uint32_t SpikeAndHold::cursch[4];
bool SpikeAndHold::sch_running;
bool SpikeAndHold::sch_loaded;

SpikeAndHold::SpikeAndHold(const uint8_t pins[NUM_VALVES]) {
    for (uint8_t i = 0; i < SpikeAndHold::NUM_VALVES; i++) {
        valve_pins[i] = pins[i];
        pinMode(valve_pins[i], OUTPUT);
    }
    sch_running = false;
    sch_loaded = false;
}

bool SpikeAndHold::setup() { return true; } // There's nothing to set up!
bool SpikeAndHold::is_functional() { return true; } // Not really any way to check if GPIO is functional or not.
void SpikeAndHold::disable() { shut_all_valves(); }
void SpikeAndHold::reset() { shut_all_valves(); }
void SpikeAndHold::single_comp_test() { 
    // TODO
}

void SpikeAndHold::pressurize_tank(VALVE_IDS tank_valve) {
    digitalWrite(valve_pins[tank_valve], OPEN);
    delay(100);
    digitalWrite(valve_pins[tank_valve], CLOSED);
}

void SpikeAndHold::load_schedule(const uint32_t sch[4]) {
    memcpy(cursch, sch, 4*sizeof(uint32_t));
}

void SpikeAndHold::execute_schedule() {
    if (!sch_loaded) return;
    sch_running = true;

    // Create 2 ms gap between valve openings, and open the ones that should be opened.
    for (uint8_t i = 0; i < 4 && sch_running; i++) {
        if (cursch[i] != 0) {
            digitalWrite(valve_pins[i + 2], OPEN);
            delay(3);
        }
    }
    
    // Order valves by firing length.
    std::vector<std::pair<uint32_t, uint8_t>> valve_firings;
    for(uint8_t i = 0; i < 4; i++) valve_firings.emplace_back(cursch[i], i);
    sort(valve_firings.begin(), valve_firings.end());

    // Convert delays to differences in delays
    for(uint8_t i = 1; i < 4; i++) {
        valve_firings.at(i).first = valve_firings.at(i - 1).first - valve_firings.at(i).first;
    }
    // Wait the appropriate amount between firings, and then close the appropriate valve.
    for(uint8_t i = 0; i < 4 && sch_running; i++) {
        auto x = valve_firings.at(i);
        delay(x.first);
        if (x.first != 0) delay(2);
        digitalWrite(valve_pins[x.second + 2], CLOSED);
    }

    // Just to be extra sure that all valves end up being closed, close them off.
    shut_all_valves();
    sch_loaded = false;
    sch_running = false;
}

void SpikeAndHold::shut_all_valves() {
    for(uint8_t i = 0; i < NUM_VALVES; i++) digitalWrite(valve_pins[i], CLOSED);
}

bool SpikeAndHold::schedule_is_loaded() { return sch_loaded; }
bool SpikeAndHold::schedule_is_running() { return sch_running; }
void SpikeAndHold::abort_schedule() { sch_running = false; }
uint32_t SpikeAndHold::time_until_execution() {
    return (uint32_t) std::max_element(cursch, cursch+4);
}

SpikeAndHold::VALVE_STATE SpikeAndHold::valve_status(VALVE_IDS valve) { return valve_states[valve]; }