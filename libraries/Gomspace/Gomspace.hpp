#ifndef GOMSPACE_HPP_
#define GOMSPACE_HPP_

#include <I2CDevice.hpp>

namespace PAN {
namespace Devices {
class Gomspace : public I2CDevice {
  public:
    /** \brief Constructs Gomspace interface on the specified wire and with the given address. */
    Gomspace(i2c_t3 &i2c_wire, uint8_t i2c_addr);

  private:
    struct __attribute__((packed))
    {
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
    } eps_hk_t;

    struct __attribute__((packed))
    {
        uint16_t vboost[3]; //! Voltage of boost converters [mV] [PV1, PV2, PV3]
        uint16_t vbatt;     //! Voltage of battery [mV]
        uint16_t curin[3];  //! Current in [mA]
        uint16_t cursun;    //! Current from boost converters [mA]
        uint16_t cursys;    //! Current out of battery [mA]
        uint16_t reserved1; //! Reserved for future use
    } eps_hk_vi_t;

    struct __attribute__((packed))
    {
        uint16_t curout[6];           //! Current out (switchable outputs) [mA]
        uint8_t output[8];            //! Status of outputs**
        uint16_t output_on_delta[8];  //! Time till power on** [s]
        uint16_t output_off_delta[8]; //! Time till power off** [s]
        uint16_t latchup[6];          //! Number of latch-ups
    } eps_hk_out_t;

    struct __attribute__((packed))
    {
        uint32_t wdt_i2c_time_left;    //! Time left on I2C wdt [s]
        uint32_t wdt_gnd_time_left;    //! Time left on I2C wdt [s]
        uint8_t wdt_csp_pings_left[2]; //! Pings left on CSP wdt
        uint32_t counter_wdt_i2c;      //! Number of WDT I2C reboots
        uint32_t counter_wdt_gnd;      //! Number of WDT GND reboots
        uint32_t counter_wdt_csp[2];   //! Number of WDT CSP reboots
    } eps_hk_wdt_t;

    struct __attribute__((packed))
    {
        uint32_t counter_boot; //! Number of EPS reboots
        int16_t temp[6];       //! Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BATT0, BATT1]
        uint8_t bootcause;     //! Cause of last EPS reset
        uint8_t battmode;      //! Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        uint8_t pptmode;       //! Mode of PPT tracker [1=MPPT, 2=FIXED]
        uint16_t reserved2;
    } eps_hk_basic_t;

    struct __attribute__((packed))
    {
        uint8_t ppt_mode;                     //! Mode for PPT [1 = AUTO, 2 = FIXED]
        uint8_t battheater_mode;              //! Mode for battheater [0 = Manual, 1 = Auto] //! Turn heater on at [degC]
        int8_t battheater_low;                //! Turn heater on at [degC]
        int8_t battheater_high;               //! Turn heater off at [degC]
        uint8_t output_normal_value[8];       //! Nominal mode output value
        uint8_t output_safe_value[8];         //! Safe mode output value
        uint16_t output_initial_on_delay[8];  //! Output switches: init with these on delays [s]
        uint16_t output_initial_off_delay[8]; //! Output switches: init with these off delays [s]
        uint16_t vboost[3];                   //! Fixed PPT point for boost converters [mV]
    } eps_config_t;

    struct __attribute__((packed))
    {
        uint16_t batt_maxvoltage;
        uint16_t batt_safevoltage;
        uint16_t batt_criticalvoltage;
        uint16_t batt_normalvoltage;
        uint32_t reserved1[2];
        uint8_t reserved2[4];
    } eps_config2_t;

    struct __attribute__((packed))
    {
        uint8_t version;
        uint8_t cmd;
        uint8_t length;
        uint8_t flags;
        uint16_t cur_lim[8];
        uint8_t cur_ema_gain;
        uint8_t cspwdt_channel[2];
        uint8_t cspwdt_address[2];
    } eps_config3_t;

    eps_hk_t hk;
    eps_hk_vi_t hk_vi;
    eps_hk_out_t hk_out;
    eps_hk_wdt_t hk_wdt;
    eps_hk_basic_t hk_basic;
    eps_config2_t config;
    eps_config2_t config2;
    eps_config3_t config3;

    void _get_hk_2();
    void _get_hk_2_vi();
    void _get_hk_2_out();
    void _get_hk_2_wdt();
    void _get_hk_2_basic();
    void _set_output(uint8_t output_byte);
    void _set_single_output(uint8_t channel, uint8_t value, int16_t time_delay);
    void _set_pv_volt(uint16_t voltage1, uint16_t voltage2, uint16_t voltage3);
    void _set_pv_auto(uint8_t mode);
    uint8_t* _set_heater(uint8_t cmd, uint8_t header, uint8_t mode);
    void _get_heater();
    void _reset_counters(uint8_t magic);
    void _reset_wdt(uint8_t magic);
    void _config_cmd(uint8_t cmd);
    void _config_get();
    void _config_set(const eps_config_t &config);
    void _hard_reset();
    void _config2_cmd(uint8_t cmd);
    void _config2_get();
    void _config2_set(const eps_config2_t &config);
    void _config3(uint8_t cmd);

    bool _ping(uint8_t value);
    void _reboot();
};
}
}

#endif