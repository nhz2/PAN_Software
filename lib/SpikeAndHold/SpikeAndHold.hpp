#ifndef SPIKE_AND_HOLD_HPP_
#define SPIKE_AND_HOLD_HPP_

#include "../Devices/Device.hpp"

namespace Devices {
    /** 
     * To use the class, update the firing schedule, and then call execute_schedule().
     * Example:
     * 
     * SpikeAndHold sph;
     * sph.setup();
     * sph.current_schedule.schedule_items[0] = {0, 0, 1};
     * sph.current_schedule.schedule_items[1] = {0, 1, 1};
     * sph.current_schedule.schedule_items[2] = {0, 2, 1};
     * sph.current_schedule.schedule_items[3] = {100, 0, 0};
     * sph.current_schedule.schedule_items[4] = {100, 1, 0};
     * sph.current_schedule.schedule_items[5] = {100, 2, 0};
     * sph.items_in_schedule = 6;
     * sph.execute_schedule();
     * **/
    class SpikeAndHold : public Device {
      public:
        static constexpr uint8_t NUM_VALVES = 6;
        enum VALVE_IDS {
            INTERTANK_MAIN = 0, // Main tank 1 to tank 2 valve
            INTERTANK_BACKUP = 1, // Backup tank 1 to tank 2 valve
            NOZZLE_1 = 2, // Nozzle valve
            NOZZLE_2 = 3, // Nozzle valve
            NOZZLE_3 = 4, // Nozzle valve
            NOZZLE_4 = 5 // Nozzle valve
        };
        enum VALVE_STATE {
            CLOSED = 0,
            OPEN = 1
        };

        uint8_t valve_status[6]; // Whether each valve is on or off.
        uint8_t unclosed_valves[6]; // Whether or not the valve was left open at the end of the last
                                    // manuever.

        struct FiringScheduleItem {
            uint32_t time; // Time, in milliseconds, at which action will be taken
            VALVE_IDS valve; // A number from 0 to 5
            VALVE_STATE state; // Open (1), or close (0)?
        };
        struct FiringSchedule {
            FiringScheduleItem *items;
        };

        static FiringScheduleItem* cursch; // The current schedule
        uint8_t num_items_sch; // Number of items in the current schedule
        static constexpr uint8_t MAX_SCHEDULE_ITEMS = 100;

        /** \brief Default constructor. Loads a set of hardcoded pins into the valve table.**/
        static constexpr uint8_t DEFAULT_VALVE_PINS[6] {3, 4, 5, 6, 9, 10};
        SpikeAndHold(const uint8_t pins[NUM_VALVES] = DEFAULT_VALVE_PINS);

        /** \brief Blocking call to execute the schedule contained within the class.
         *  \returns True if the schedule's execution was completed. False if the schedule could not
         *  be executed (either because it was incorrect or because an I/O error caused a failure.) **/
        bool execute_schedule();

        /** \brief Shut all valves. **/
        void shut_all_valves();

        /** \brief Aborts the currently running schedule. **/
        void abort_schedule();

        /** \brief Checks if there is currently a schedule operating on the valves. **/
        bool schedule_is_running();

        /** \brief Indicates how long there is until the schedule is expected to be completed. **/
        uint32_t time_until_execution();
      private:
        uint8_t valve_pins[NUM_VALVES]; // # of GPIO pin that valve is connected to.

        /** \brief Ensures a schedule is valid, by checking the following things:
         * - All valve pins referred to are valid valve pins. 
         * - All valves that are opened are also closed. 
         * - The final opening/closing is not beyond the overflow limit for micros().
         *   This is not a strictly necessary condition on the schedule, but is a good constraint to
         *   pose to eliminate any chance of overflow errors. **/
        bool _schedule_is_correct();

        // Greates a 2 ms gap between valve openings, if needed.
        void _fix_schedule();

        bool sch_running; // Whether or not a schedule is currently being executed.
        uint32_t sch_length; // Current schedule length in milliseconds.
                             // Should be zero if there is no manuever currently being executed.
        uint32_t start_time; // Time, in milliseconds, at which the current schedule was started.
                             // Should be zero if there is no manuever currently being executed.
    };
}

#endif