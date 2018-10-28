#include "Arduino.h"
#include <climits>
#include "../Devices/Device.hpp"
#include "SpikeAndHold.hpp"

using namespace Devices;

SpikeAndHold::SpikeAndHold(const uint8_t pins[NUM_VALVES]) {
    for (uint8_t i = 0; i < SpikeAndHold::NUM_VALVES; i++) {
        valve_pins[i] = pins[i];
    }
    cursch.items = (FiringScheduleItem*) malloc(MAX_SCHEDULE_ITEMS * sizeof(FiringScheduleItem));
    ms_till_done = 0;
}

SpikeAndHold::SpikeAndHold(const uint8_t pins[SpikeAndHold::NUM_VALVES]) {
    for (uint8_t i = 0; i < SpikeAndHold::NUM_VALVES; i++) {
        valve_pins[i] = pins[i];
    }
}

bool SpikeAndHold::execute_schedule() {
    _fix_schedule();

    sch_running = true;
    sch_length = cursch[num_items_sch - 1].time;
    start_time = millis();

    for(uint8_t i = 0; i < num_items_sch && sch_running; i++) {
        
        uint8_t valve_gpio_pin = valve_pins[cursch[i].valve];
        uint8_t old_valve_state = valve_status[cursch[i].valve];
        uint8_t new_valve_state = cursch[i].state;
        digitalWrite(valve_gpio_pin, new_valve_state);

        uint32_t dt;
        if (i + 1 != num_items_sch) dt = cursch[i + 1].time - cursch[i].time;
        else dt = 0;
        delay(dt);
    }
    // Turn all valves off in case schedule forgot to do this somehow
    shut_all_valves();

    sch_running = false;
    sch_length = 0;
    num_items_sch = 0;
    start_time = 0;

    return true;
}

void SpikeAndHold::shut_all_valves() {
    for(uint8_t i = 0; i < NUM_VALVES; i++) { 
        digitalWrite(valve_pins[i], LOW);
    }
}

void SpikeAndHold::_fix_schedule() {
    for(uint8_t i = 0; i < num_items_sch - 1; i++) {
        uint32_t dt = cursch[i + 1].time - cursch[i].time;
        if (dt < 2) {
            for(uint8_t j = i + 1; j < num_items_sch; j++) cursch[j].time += (2 - dt);
        }
    }
}

bool SpikeAndHold::schedule_is_running() { return sch_running; }
void SpikeAndHold::abort_schedule() { sch_running = false; }
uint32_t SpikeAndHold::time_until_execution() { 
    if (!sch_running) return 0;
    else return sch_length - (millis() - start_time); 
}