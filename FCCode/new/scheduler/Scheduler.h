#include <map>
#include <iostream>

namespace PAN {
class Scheduler {
  private:
    std::map<long, Event> events; // Map key is timestamp of when event is scheduled
  public:
    struct Event {
        long timestamp_event_added;
        // TODO add more parameters by defining all events, e.g.
        // Type of action
        // Device action
        // Action parameters
        // 
    };
    /** \brief Adds an event to the proper position in the schedule.
     * \param t The time at which the event will be executed.
     * \param e The event to add.
     * \throws Invalid argument if an event already exists in the scheduler at the time t.
     * \throws Overflow error if there are too many events in the scheduler (>255).
     * **/
    void add_event(long t, Event e);
    /** \brief Returns next event after a specified time.
     * \param t The specified time.
     * \returns A reference to the event if such an event exists. If the time is beyond all 
     * events in the scheduler, a null reference is returned.
     * **/
    const Event &get_event(long t);

    /** \brief Removes an event from the schedule.
     * \param t The time at which the event was supposed to be executed.
     * **/
    void remove_event(long t);
    /** \brief Removes the earliest (i.e. next) event from the schedule. **/
    void remove_next_event();
    /** \brief Removes all events from the schedule. **/
    void remove_all_events();
    /** \brief Returns number of events currently on the schedule.
     *  \returns Number of events. Since the return type is an 8-bit integer, this
     * sets a maximum of 255 events in the schedule. **/
    uint8_t number_of_events();
};
}