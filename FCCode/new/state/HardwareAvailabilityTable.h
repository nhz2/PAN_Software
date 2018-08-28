namespace PAN {
namespace Devices {
// Hardware Availability Table
class HAVT {
  private:
    std::array<std::string, NUMBER_OF_DEVICES> devices = {"PAN ADCS", "Piksi", "GomSpace", "Quake"};
    struct HardwareState
    {
        bool responding; // Is part responding to communications?
        bool functional; // Is part functional? (e.g. is it returning appropriate data)
    };
    std::array<HardwareState, NUMBER_OF_DEVICES> havt; // Hardware Availability Table
  public:
    const uint8_t NUMBER_OF_DEVICES = 4;

    /** Returns true if device was in list (and therefore status was changed
     * successfully), false otherwise. **/
    bool set_device_responding(std::string device_name, bool is_responding);

    /** Returns true if device was in list (and therefore status was changed 
     * successfully), false otherwise. **/
    bool set_device_functional(std::string device_name, bool is_functional);

    /** Returns an (immutable) list of devices. **/
    std::array<std::string, NUMBER_OF_DEVICES> const &get_device_list();
};
}
}