#include <Arduino.h>
#include <i2c_t3_pan/i2c_t3_pan.h>
#include <Gomspace.hpp>
using namespace Devices;

#define HK_READ_TESTING
// #define CONFIG_READ_TESTING
// #define CONFIG2_READ_TESTING
// #define HEATER_READ_TESTING
// #define OUTPUT_TESTING
// #define RESET_COUNTERS_TESTING

Gomspace gs(Wire1, Gomspace::ADDRESS);
bool setup_result;

void setup() {
    Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
    Serial.begin(9600);
    delay(100);
    setup_result = gs.setup();
    Serial.println("Successfully set up Gomspace.");
}

void loop() {
    #ifdef HK_READ_TESTING
        if (gs.get_hk()) {
            Gomspace::eps_hk_t* hk = gs.hk;

            Serial.printf("boost converter voltages (mV): %d,%d,%d\n", hk->vboost[0], hk->vboost[1], hk->vboost[2]);
            Serial.printf("vbattery (mV): %d\n", hk->vbatt);
            Serial.printf("current in (mA): %d,%d,%d\n", hk->curin[0], hk->curin[1], hk->curin[2]);
            Serial.printf("current from boost converters (mA): %d\n", hk->cursun);
            Serial.printf("current out from battery (mA): %d\n", hk->cursys);
            Serial.printf("current out from outputs: %d,%d,%d,%d,%d,%d\n", hk->curout[0], hk->curout[1], hk->curout[2],
                hk->curout[3], hk->curout[4], hk->curout[5]);
            Serial.printf("are outputs on?: %d,%d,%d,%d,%d,%d,%d,%d\n", hk->output[0], hk->output[1], hk->output[2],
                hk->output[3], hk->output[4], hk->output[5], hk->output[6], hk->output[7]);
            Serial.printf("time left on i2c wdt: %d\n", hk->wdt_i2c_time_left);
            Serial.printf("number of reboots due to i2c: %d\n", hk->counter_wdt_i2c);
            Serial.printf("number of reboots of EPS: %d\n", hk->counter_boot);
            Serial.printf("temperature readings: %d,%d,%d,%d,%d,%d\n", hk->temp[0], hk->temp[1],hk->temp[2], hk->temp[3],
                hk->temp[4], hk->temp[5]);
            Serial.printf("cause of last reboot: %d\n", hk->bootcause);
            Serial.printf("current battery mode: %d\n", hk->battmode);
            Serial.printf("current ppt mode: %d\n", hk->pptmode);
        }
        else {
            Serial.println("Failed to fetch housekeeping data.");
        }
        delay(4000);
    #endif
    #ifdef CONFIG_READ_TESTING
        if (gs.config_get()) {
            Gomspace::eps_config_t* config = &gs.gspace_config;

            Serial.printf("PPT mode: %d\n", config->ppt_mode);
            Serial.printf("Battery heater mode: %d\n", config->battheater_mode);
            Serial.printf("Battery heater low: %d\n", config->battheater_low);
            Serial.printf("Battery heater high: %d\n", config->battheater_high);
            Serial.printf("Output normal values: %d,%d,%d,%d,%d,%d,%d,%d\n", config->output_normal_value[0],
            config->output_normal_value[1], config->output_normal_value[2], config->output_normal_value[3],
            config->output_normal_value[4], config->output_normal_value[5], config->output_normal_value[6],
            config->output_normal_value[7]);
            Serial.printf("Output safe values: %d,%d,%d,%d,%d,%d,%d,%d\n", config->output_safe_value[0],
            config->output_safe_value[1], config->output_safe_value[2], config->output_safe_value[3],
            config->output_safe_value[4], config->output_safe_value[5], config->output_safe_value[6],
            config->output_safe_value[7]);
            Serial.printf("Output initial on delay: %d,%d,%d,%d,%d,%d,%d,%d\n", config->output_initial_on_delay[0],
            config->output_initial_on_delay[1], config->output_initial_on_delay[2], config->output_initial_on_delay[3],
            config->output_initial_on_delay[4], config->output_initial_on_delay[5], config->output_initial_on_delay[6],
            config->output_initial_on_delay[7]);
            Serial.printf("Output initial off delay: %d,%d,%d,%d,%d,%d,%d,%d\n", config->output_initial_off_delay[0],
            config->output_initial_off_delay[1], config->output_initial_off_delay[2], config->output_initial_off_delay[3],
            config->output_initial_off_delay[4], config->output_initial_off_delay[5], config->output_initial_off_delay[6],
            config->output_initial_off_delay[7]);
            Serial.printf("Boost converter voltages: %d,%d,%d\n", config->vboost[0], config->vboost[1], config->vboost[2]);
        }
        else {
            Serial.println("Failed to fetch config.");
        }
        delay(4000);
    #endif
    #ifdef CONFIG2_READ_TESTING
        if (gs.config2_get()) {
            Gomspace::eps_config2_t* config2 = &gs.gspace_config2;

            Serial.printf("Battery maximum voltage: %d\n", config2->batt_maxvoltage);
            Serial.printf("Battery critical voltage: %d\n", config2->batt_criticalvoltage);
            Serial.printf("Battery normal voltage: %d\n", config2->batt_normalvoltage);
            Serial.printf("Battery safe voltage: %d\n", config2->batt_safevoltage);
        }
        else {
            Serial.println("Failed to fetch config.");
        }
        delay(4000);
    #endif
    #ifdef HEATER_READ_TESTING
        Serial.printf("Heater status code: %d\n", gs.get_heater());
        delay(4000);
    #endif
    #ifdef OUTPUT_TESTING
        //Serial.println(gs.set_output(0b00111111));
        Gomspace::eps_hk_t* hk = gs.hk;
        gs.set_single_output(0, 0);
        gs.set_single_output(1, 1);
        gs.set_single_output(2, 1);
        gs.set_single_output(3, 1);
        gs.set_single_output(4, 1);
        gs.set_single_output(5, 1);
        gs.get_hk();
        Serial.printf("outputs after off: %d,%d,%d,%d,%d,%d,%d,%d\n", hk->output[0], hk->output[1], hk->output[2],
            hk->output[3], hk->output[4], hk->output[5], hk->output[6], hk->output[7]);
        delay(1000);
        gs.set_single_output(0, 1);
        gs.set_single_output(1, 0);
        gs.set_single_output(2, 0);
        gs.set_single_output(3, 0);
        gs.set_single_output(4, 0);
        gs.set_single_output(5, 0);
        gs.get_hk();
        Serial.printf("outputs after on: %d,%d,%d,%d,%d,%d,%d,%d\n", hk->output[0], hk->output[1], hk->output[2],
            hk->output[3], hk->output[4], hk->output[5], hk->output[6], hk->output[7]);
        delay(3000);
        delay(1000);
    #endif
    #ifdef RESET_COUNTERS_TESTING
        Serial.println(gs.reset_counters());
        Gomspace::eps_hk_t* hk = gs.hk;
        Serial.println(gs.get_hk());
        Serial.printf("time left on i2c wdt: %d\n", hk->wdt_i2c_time_left);
        Serial.printf("number of reboots due to i2c: %d\n", hk->counter_wdt_i2c);
        delay(4000);
    #endif
}