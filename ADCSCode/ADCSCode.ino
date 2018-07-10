
#include <i2c_t3.h>
#include <I2CDevice.hpp>
#include <PololuIMU.hpp>
#include <ADS1115.hpp>

/*! If defined the debug serial output is active */
#define DEBUG

/*! Namespace containing code to control the sun sensor assembly. The code in
 *  this section is responsible for reading, processing, and monitoring the data
 *  and behavior of the sun sensors.
 */
namespace ssa {

/*! Configures how many channels are read of each ADC for each call to
 *  ssa::read(). This has a max value of 4 because there are 4 channels on each
 *  ADC.
 */
#define SSA_READS_PER_CALL 1
/*! Defines standard ADC timeout in milliseconds */
#define SSA_ADC_TIMEOUT 1000
/*! Toggles SSA specific debug as on or off */
#ifdef DEBUG
#define SSA_DEBUG
#endif

  /*! ADC objects */
  static ADS1115 adc1(Wire,  ADS1115_ADDR_GND, 11);
  static ADS1115 adc2(Wire,  ADS1115_ADDR_VDD, 12);
  static ADS1115 adc3(Wire,  ADS1115_ADDR_SDA, 14);
  static ADS1115 adc4(Wire1, ADS1115_ADDR_GND, 13);
  static ADS1115 adc5(Wire1, ADS1115_ADDR_VDD, 20);

  /*! Sun sensor reading stored data structure */
  static int16_t data[20];

  /*! Array tracking sun sensor failures and whether a sensor is disabled */
  static unsigned int broken[5];

  /*! Initializes ADC object settings and sets ssa namespace data members to
   *  their default values. It is assumed that the wires used by the ADCs in the
   *  ssa namespace have already been initialized.
   */
  void init() {
    // ADC configuration helper function
    auto config_adc = [](ADS1115 &adc) {
      adc.i2c_set_timeout(SSA_ADC_TIMEOUT);
      adc.set_gain(ADS1115_GAIN_TWO_THIRDS);
      adc.set_sample_rate(ADS1115_860_SPS);
    };
    // Configure all of the adc's
    config_adc(adc1);
    config_adc(adc2);
    config_adc(adc3);
    config_adc(adc4);
    config_adc(adc5);
    // Populate initial measurements
    for(int i = 0; i < 4; i++) {
      adc1.start_read(i);
      adc2.start_read(i);
      adc3.start_read(i);
      adc4.start_read(i);
      adc5.start_read(i);
    }
  }

  void read() {

  }

}

namespace mtr {

  // TODO : MTR stuff goes here

}

void setup() {
  // Initialize all I2C busses
  Wire.begin();
  Wire1.begin();
#ifdef DEBUG
  Serial.begin(9600);
#endif
}

void loop() {

}
