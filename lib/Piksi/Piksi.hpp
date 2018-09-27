#ifndef PIKSI_HPP_
#define PIKSI_HPP_

#include "../Devices/Device.hpp"
#include "libsbp/sbp.h"
#include "libsbp/navigation.h"
#include "libsbp/observation.h"
#include "libsbp/settings.h"
#include "libsbp/system.h"
#include "libsbp/piksi.h"
#include "libsbp/user.h"

namespace Devices {
class Piksi : public Device {
  public:
    // Standard device functions
    bool setup() override;
    bool is_functional() override;
    void reset() override;
    void disable() override;
    void single_comp_test() override;

    /** \brief Gets GPS time. 
     *  \return GPS time, as a libsbp struct. **/
    msg_gps_time_t const &get_gps_time();
    /** \brief Gets Dilution of Precision. 
     *  \return Dilution of precision, as a libsbp struct. **/
    msg_dops_t const &get_dops();
    /** \brief Gets satellite position in ECEF coordinates.
     *  \return Satellite position in ECEF coordinates, as a libsbp struct. **/
    msg_pos_ecef_t const &get_pos_ecef();
    /** \brief Gets satellite position in ECEF coordinates relative to base station.
     *  \return Satellite position in ECEF coordinates relative to base station, as a libsbp struct. **/
    msg_baseline_ecef_t const &get_baseline_ecef();
    /** \brief Gets satellite velocity in ECEF coordinates.
     *  \return Satellite position in ECEF coordinates, as a libsbp struct. **/
    msg_vel_ecef_t const &get_vel_ecef();
    /** \brief Gets base station position in ECEF coordinates.
     *  \return Base station position in ECEF coordinates, as a libsbp struct. **/
    msg_base_pos_ecef_t const &get_base_pos_ecef();
    /** \brief Reads current settings in Piksi RAM.
     *  \return Current settings in Piksi RAM, as a libsbp struct. **/
    msg_settings_read_resp_t const &get_settings_read_resp();
    /** \brief Reads status flags of Piksi (i.e. the "heartbeat").
     *  \return Status flags of Piksi, as a libsbp struct. **/
    msg_heartbeat_t const &get_heartbeat();
    /** \brief Reads UART channel health of Piksi.
     *  \return UART channel health, as a libsbp struct. **/
    msg_uart_state_t const &get_uart_state();
    /** \brief Reads user data sent from MRO radio (probably);
     *  \return MRO radio data, as a libsbp struct. **/
    msg_user_data_t const &get_user_data();

    /** \brief Saves the data settings to flash. **/
    void settings_save();
    /** \brief Writes the desired settings to the Piksi's RAM.
     * \param Struct containing setting changes for the Piksi. **/
    void settings_write(const msg_settings_write_t &settings);
    /** \brief Resets Piksi. **/
    void piksi_reset();
    /** \brief Sends custom user data to the Piksi. This data is
     * (probably?) passed through the MRO radio to the other Piksi.
     * \param User data, as an array of (maximally 255) bytes. **/
    void send_user_data(const msg_user_data_t &data);
  private:
    // Internal values required by libsbp. See sbp.c
    sbp_state_t _sbp_state;
    static sbp_msg_callbacks_node_t _gps_time_callback_node;
    static sbp_msg_callbacks_node_t _dops_callback_node;
    static sbp_msg_callbacks_node_t _pos_ecef_callback_node;
    static sbp_msg_callbacks_node_t _baseline_ecef_callback_node;
    static sbp_msg_callbacks_node_t _vel_ecef_callback_node;
    static sbp_msg_callbacks_node_t _base_pos_ecef_callback_node;
    static sbp_msg_callbacks_node_t _settings_read_resp_callback_node;
    static sbp_msg_callbacks_node_t _heartbeat_callback_node;
    static sbp_msg_callbacks_node_t _uart_state_callback_node;
    static sbp_msg_callbacks_node_t _user_data_callback_node;

    // Callback functions required by libsbp for read functions. See sbp.c
    void _gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _settings_read_resp_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    void _user_data_callback(u16 sender_id, u8 len, u8 msg[], void *context);

    // Registers all callbacks with libsbp.
    void _register_all_callbacks();

    // Value containers.
    msg_gps_time_t _gps_time;
    msg_dops_t _dops;
    msg_pos_ecef_t _pos_ecef;
    msg_baseline_ecef_t _baseline_ecef;
    msg_vel_ecef_t _vel_ecef;
    msg_base_pos_ecef_t _base_pos_ecef;
    msg_settings_read_resp_t _settings_read_resp;
    msg_heartbeat_t _heartbeat;
    msg_uart_state_t _uart_state;
    msg_user_data_t _user_data;
};
}


#endif