#include <map>
#include <iostream>
#include "Scheduler.h"

using namespace PAN;

void Scheduler::add_event(long t, Scheduler::Event e) {
    if (number_of_events() == 255)
        throw std::overflow_error("Too many events in scheduler");
    try {
        events.at(t);
        throw std::invalid_argument("Event already exists in scheduler at the same time");
    }
    catch(std::exception excpn) {
        // Proceed with adding event.
        events.insert(t, e);
    }
}

const auto &Scheduler::get_event(long t) {
    return events.at(t);
}

void Scheduler::remove_event(long t) {
    events.remove(t);
}

void Scheduler::remove_next_event() {
    auto it = events.begin(); // Iterator pointing to first event in schedule
    events.erase(it);
}

void Scheduler::remove_all_events() {
    events.clear();
}

uint8_t Scheduler::number_of_events() {
    return events.size();
}