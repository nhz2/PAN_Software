#include <Arduino.h>
#include "../Devices/Device.hpp"

namespace Devices {
    class SpikeAndHold : public Device {
      public:
        static const uint8_t NUM_VALVES = 6; // [0]: Main tank 1 to tank 2 valve.
                                             // [1]: Backup tank 1 to tank 2 valve.
                                             // [2]: Nozzle valve.
                                             // [3]: Nozzle valve.
                                             // [4]: Nozzle valve.
                                             // [5]: Nozzle valve.

        struct FiringSchedule {
            uint32_t **openings; // Map of valves that are opened, indexed (sorted) by time
            uint32_t **closings; // Map of valves that are closed, indexed (sorted) by time
        };
        /** \brief Default constructor. Loads a set of hardcoded pins into the valve table.**/
        SpikeAndHold();
        /** \brief Argumented constructor. Loads the set of specified pins into the valve table.**/
        SpikeAndHold(const uint8_t pins[NUM_VALVES]);

        /** \brief Blocking call to fire the thrusters for the specified durations.
         *  \param durations An integer array containing how long, in milliseconds, to fire the valves.
         *  If the value is zero, the valve will not be fired. **/
        bool fire_thrusters(const FiringSchedule &schedule);

      private:
        uint8_t valve_pins[NUM_VALVES]; // # of GPIO pin that valve is connected to.

        /** \brief Ensures a schedule is valid, by checking the following things:
         * - All valve pins referred to are valid valve pins. 
         * - All valves that are opened are also closed. 
         * - The final opening/closing is not beyond the overflow limit for micros().
         *   This is not a strictly necessary condition on the schedule, but is a good constraint to
         *   pose to eliminate any chance of overflow errors. 
         *  \param schedule The schedule to check. **/
        bool _schedule_is_correct(const FiringSchedule &schedule);
    };
}