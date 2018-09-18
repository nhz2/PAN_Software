#include <Device.hpp>

namespace Devices {
class MCP4162 : public Device {
  private:
    SPIClass &spi;
    uint8_t ss; // Slave select pin

    float _min_resistance; // Min resistance for this particular temperature regime
    float _max_resistance; // Max resistance for this particular temperature regime
    float _delta_resistance; // Delta-resistance per change in wiper value
    float temperature;

    float resistance;
    float _wiper_setting;
  public:
    /** \brief Constructor. **/
    MCP4162(SPIClass &s, uint8_t slave_select);

    // Device functions
    bool dev_setup();
    bool dev_is_functional();
    void dev_reset();
    void dev_disable();
    String dev_sc_test();

    /** \brief Set temperature. This will tell the set_resistance() and get_resistance()  how to appropriately set the wiper. */
    void set_temperature(float t);
    /** \brief Set resistance to a desired value r, if r is within the available resistance range.
     * \param r The resistance, in ohms, to be set. We must have ~125 < r < ~5000.
     */
    void set_resistance(float r);
    /** \brief Get the currently set resistance value. **/
    float get_resistance();

    /** \brief Manually set the wiper. This is useful for creating a resistance profile.
     * \param value The wiper value to set, from 0 to 255.
     */
    void set_wiper(uint8_t value);
};
}