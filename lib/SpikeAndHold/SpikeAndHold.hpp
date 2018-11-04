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
        static constexpr uint8_t TANK_VALVE_FIRING_TIME = 100; // Amount of time to briefly open the intertank valve, in milliseconds.

        /** \brief Default constructor. Loads a set of hardcoded pins into the valve table.**/
        static uint8_t DEFAULT_VALVE_PINS[6];
        SpikeAndHold(const uint8_t pins[NUM_VALVES] = DEFAULT_VALVE_PINS);

        bool setup() override;
        bool is_functional() override;
        void disable() override;
        void reset() override;
        void single_comp_test() override;

        /** Pressurizes tank by opening a specified valve for 100 ms. **/
        void pressurize_tank(VALVE_IDS valve = INTERTANK_MAIN);

        /** \brief Specify a pointer to a schedule that will be executed at some future time. **/
        void load_schedule(const uint32_t sch[4]);
        /** \brief Execute the schedule specified by the pointer. **/
        void execute_schedule();

        /** \brief Shut all valves. **/
        void shut_all_valves();

        /** \brief Aborts the currently running schedule. **/
        void abort_schedule();

        /** \brief Checks if there is currently a schedule loaded into the driver. **/
        bool schedule_is_loaded();

        /** \brief Checks if there is currently a schedule operating on the valves. **/
        bool schedule_is_running();

        /** \brief Lists valve statuses (i.e. whether or not a valve is open or closed.)
         *  \param Valve to check.
         *  \returns Whether valve is open or closed. **/
        VALVE_STATE valve_status(VALVE_IDS valve);

        /** \brief Indicates how long there is until the schedule is expected to be completed. **/
        uint32_t time_until_execution();
      private:
        static uint8_t valve_pins[NUM_VALVES]; // # of GPIO pin that valve is connected to.
        static VALVE_STATE valve_states[NUM_VALVES]; // Current state of valves (on or off), according to system.
        static uint32_t cursch[4]; // Schedule to be executed at a later time. 
        static bool sch_running; // Whether or not a schedule is currently being executed.
        static bool sch_loaded; // Whether or not a schedule is currently specified.
    };
}

#endif