#include "../Devices/I2CDevice.hpp"

#ifndef GOMSPACE_HPP_
#define GOMSPACE_HPP_

namespace Devices {
class Gomspace : public I2CDevice {
  public:
    static const uint8_t ADDRESS = 0x02; /**< I2C address of Gomspace device **/

    /**< "Housekeeping" data struct; contains Gomspace state information. */
    struct __attribute__((packed)) eps_hk_t {
        uint16_t vboost[3];            //! Voltage of boost converters [mV] [PV1, PV2, PV3] //! Voltage of battery [mV]
        uint16_t vbatt;                //! Voltage of battery [mV]
        uint16_t curin[3];             //! Current in [mA]
        uint16_t cursun;               //! Current from boost converters [mA]
        uint16_t cursys;               //! Current out of battery [mA]
        uint16_t reserved1;            //! Reserved for future use
        uint16_t curout[6];            //! Current out (switchable outputs) [mA]
        uint8_t output[8];             //! Status of outputs**
        uint16_t output_on_delta[8];   //! Time till power on** [s]
        uint16_t output_off_delta[8];  //! Time till power off** [s]
        uint16_t latchup[6];           //! Number of latch-ups
        uint32_t wdt_i2c_time_left;    //! Time left on I2C wdt [s]
        uint32_t wdt_gnd_time_left;    //! Time left on I2C wdt [s]
        uint8_t wdt_csp_pings_left[2]; //! Pings left on CSP wdt
        uint32_t counter_wdt_i2c;      //! Number of WDT I2C reboots
        uint32_t counter_wdt_gnd;      //! Number of WDT GND reboots
        uint32_t counter_wdt_csp[2];   //! Number of WDT CSP reboots
        uint32_t counter_boot;         //! Number of EPS reboots
        int16_t temp[6];               //! Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BP4a, BP4b]
        uint8_t bootcause;             //! Cause of last EPS reset
        uint8_t battmode;              //! Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        uint8_t pptmode;               //! Mode of PPT tracker [1=MPPT, 2=FIXED]
        uint16_t reserved2;
    };

    /**< Config data struct; contains output/heater configurations and PPT configuration. */
    struct __attribute__((packed)) eps_config_t {
        uint8_t ppt_mode;                     //! Mode for PPT [1 = AUTO, 2 = FIXED]
        uint8_t battheater_mode;              //! Mode for battheater [0 = Manual, 1 = Auto]
        int8_t battheater_low;                //! Turn heater on at [degC]
        int8_t battheater_high;               //! Turn heater off at [degC]
        uint8_t output_normal_value[8];       //! Nominal mode output value
        uint8_t output_safe_value[8];         //! Safe mode output value
        uint16_t output_initial_on_delay[8];  //! Output switches: init with these on delays [s]
        uint16_t output_initial_off_delay[8]; //! Output switches: init with these off delays [s]
        uint16_t vboost[3];                   //! Fixed PPT point for boost converters [mV]
    };

    /**< Config2 data struct; contains battery voltage level definitionss. */
    struct __attribute__((packed)) eps_config2_t {
        uint16_t batt_maxvoltage;
        uint16_t batt_safevoltage;
        uint16_t batt_criticalvoltage;
        uint16_t batt_normalvoltage;
        uint32_t reserved1[2];
        uint8_t reserved2[4];
    };

    /**< Config3 data struct: contains output current limits. */
    struct __attribute__((packed)) eps_config3_t {
        uint8_t version;
        uint8_t cmd;
        uint8_t length;
        uint8_t flags;
        uint16_t cur_lim[8];
        uint8_t cur_ema_gain; // TODO what is this?
        uint8_t cspwdt_channel[2];
        uint8_t cspwdt_address[2];
    };

    /** \brief Constructs Gomspace interface on the specified wire and with the given address. */
    Gomspace(i2c_t3 &i2c_wire, uint8_t i2c_addr);

    // Device functions
    bool setup() override;
    void reset() override;
    void single_comp_test() override;
    bool i2c_ping() override;

    /** \brief Get full housekeeping data struct.
     *  \return Housekeeping data struct. */
    const eps_hk_t &get_hk_2();
    /** \brief Set output channels on or off.
     *  \param Output byte that masks channels. */
    void set_output(uint8_t output_byte);
    /** \brief Set a single output on or off, with an optional time delay.
     *  \param Channel to set on or off. (See NanoPower documentation to see how channel numbers correspond to outputs.)
     *  \param Whether to set the channel on or off.
     *  \param Time delay for change, in seconds. */
    void set_single_output(uint8_t channel, uint8_t value, int16_t time_delay = 0);
    /** \brief Set voltage of photovoltaic inputs.
     * \param Voltage of input 1, in mV.
     * \param Voltage of input 2, in mV.
     * \param Voltage of input 3, in mV. */
    void set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3);
    /** \brief Set power point mode (PPT).
     *  \param Which mode to use. See NanoPower documentation for available modes. */
    void set_pv_auto(uint8_t mode);
    /** \brief Set heater values.
     *  \param Heater # to control. 0 = BP4, 1 = onboard, 2 = both.
     *  \param Mode for heater (0 = OFF, 1 = ON). */
    void set_heater(uint8_t heater, uint8_t mode);
    /** \brief Get heater status.
     *  \return 0 = both heaters off, 1 = BP4 heater is on, 
     *   2 = Onboard heater is on, 3 = both are on. */
    uint8_t get_heater();
    /** \brief Reset boot and WDT counters. */
    void reset_counters();
    /** \brief Reset I2C watchdog timer. */
    void reset_wdt();
    /** \brief Restores NanoPower to default configuration. */
    void restore_default_config();
    /** \brief Get EPS configuration as a struct.
     *  \return Address of EPS configuration struct. */
    const eps_config_t &config_get();
    /** \brief Set EPS configuration.
     *  \param EPS configuration to set. */
    void config_set(const eps_config_t &config);
    /** \brief Hard reset the Gomspace (and power-cycle all outputs). */
    void hard_reset();
    /** \brief Restores config2 to default config2. */
    void restore_default_config2();
    /** \brief Get config2 as a struct. */
    const eps_config2_t &config2_get();
    /** \brief Set config2.
     *  \param Struct of config2 data to set. */
    void config2_set(const eps_config2_t &config);
    /** \brief TODO DOCUMENTATION */
    void config3(const eps_config3_t &c);

    /** \brief Send a ping to the NanoPower unit.
     *  \value The value to ping with. The device should send this value back.
     *  \return True if the Gomspace replied with the same code, false otherwise. */
    bool ping(uint8_t value);
    uint8_t ping_debug(uint8_t value);
    /** \brief Reboot Gomspace. */
    void reboot();
  private:
    // See struct documentation above for more information
    eps_hk_t hk;
    eps_config_t gspace_config;
    eps_config2_t gspace_config2;
    eps_config3_t gspace_config3;
};
}

#endif
