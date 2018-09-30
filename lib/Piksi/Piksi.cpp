#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "../Devices/Device.hpp"
#include "libsbp/sbp.h"
#include "libsbp/navigation.h"
#include "libsbp/observation.h"
#include "libsbp/settings.h"
#include "libsbp/system.h"
#include "libsbp/piksi.h"
#include "libsbp/user.h"
#include "Piksi.hpp"

using namespace Devices;

bool Piksi::setup() {
    sbp_state_init(&_sbp_state);

    // Register all necessary callbacks for data reads--specification provided in sbp.c
    sbp_register_callback(&_sbp_state, SBP_MSG_GPS_TIME, 
        (sbp_msg_callback_t) &Piksi::_gps_time_callback, nullptr, &_gps_time_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_DOPS, 
        (sbp_msg_callback_t) &Piksi::_dops_callback, nullptr, &_dops_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_POS_ECEF, 
        (sbp_msg_callback_t) &Piksi::_pos_ecef_callback, nullptr, &_pos_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASELINE_ECEF, 
        (sbp_msg_callback_t) &Piksi::_baseline_ecef_callback, nullptr, &_baseline_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_VEL_ECEF, 
        (sbp_msg_callback_t) &Piksi::_vel_ecef_callback, nullptr, &_vel_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF, 
        (sbp_msg_callback_t) &Piksi::_base_pos_ecef_callback, nullptr, &_base_pos_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_SETTINGS_READ_RESP, 
        (sbp_msg_callback_t) &Piksi::_settings_read_resp_callback, nullptr, &_settings_read_resp_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, 
        (sbp_msg_callback_t) &Piksi::_heartbeat_callback, nullptr, &_heartbeat_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_UART_STATE, 
        (sbp_msg_callback_t) &Piksi::_uart_state_callback, nullptr, &_uart_state_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_USER_DATA, 
        (sbp_msg_callback_t) &Piksi::_user_data_callback, nullptr, &_user_data_callback_node);

    return true;
}

bool Piksi::is_functional() { return get_heartbeat().flags == 0; }
void Piksi::reset() { piksi_reset(); }

// Cannot disable Piksi, since Piksi is integral to the system. So this function is left unimplemented.
void Piksi::disable() { }

void Piksi::single_comp_test() { /** TODO **/ }

msg_gps_time_t const &Piksi::get_gps_time() { return _gps_time; }
msg_dops_t const &Piksi::get_dops()  { return _dops; }
msg_pos_ecef_t const &Piksi::get_pos_ecef()  { return _pos_ecef; }
msg_baseline_ecef_t const &Piksi::get_baseline_ecef()  { return _baseline_ecef; }
msg_vel_ecef_t const &Piksi::get_vel_ecef()  { return _vel_ecef; }
msg_base_pos_ecef_t const &Piksi::get_base_pos_ecef()  { return _base_pos_ecef; }
msg_settings_read_resp_t const &Piksi::get_settings_read_resp()  { return _settings_read_resp; }
msg_heartbeat_t const &Piksi::get_heartbeat()  { return _heartbeat; }
msg_uart_state_t const &Piksi::get_uart_state()  { return _uart_state; }
msg_user_data_t const &Piksi::get_user_data()  { return _user_data; }

void Piksi::settings_save()  {
    sbp_send_message(&_sbp_state, SBP_MSG_SETTINGS_SAVE, SBP_SENDER_ID, 
        0, nullptr, (u32 (*)(u8 *buff, u32 n, void *context)) &Piksi::_uart_write);
}
void Piksi::settings_write(const msg_settings_write_t &settings)  {
    sbp_send_message(&_sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, 
        sizeof(msg_settings_write_t), (uint8_t*) &settings, (u32 (*)(u8 *buff, u32 n, void *context)) &Piksi::_uart_write);
}
void Piksi::piksi_reset()  {
    sbp_send_message(&_sbp_state, SBP_MSG_RESET, SBP_SENDER_ID, 
        0, nullptr, (u32 (*)(u8 *buff, u32 n, void *context)) &Piksi::_uart_write);
}
void Piksi::send_user_data(const msg_user_data_t &data)  {
    sbp_send_message(&_sbp_state, SBP_MSG_USER_DATA, SBP_SENDER_ID,
        sizeof(msg_user_data_t), (uint8_t*) &data, (u32 (*)(u8 *buff, u32 n, void *context)) &Piksi::_uart_write);
}

u32 Piksi::_uart_write(u8 *buff, u32 n, void *context) {
    u32 i;
    for(i = 0; i < n; i++) {
        if (Serial.write(buff[i]) == 0) break;
    }
    return i;
}

void Piksi::_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_gps_time), msg, sizeof(msg_gps_time_t));
}
void Piksi::_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_dops), msg, sizeof(msg_dops_t));
}
void Piksi::_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_pos_ecef), msg, sizeof(msg_pos_ecef_t));
}
void Piksi::_baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_baseline_ecef), msg, sizeof(msg_baseline_ecef_t));
}
void Piksi::_vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_vel_ecef), msg, sizeof(msg_vel_ecef_t));
}
void Piksi::_base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_base_pos_ecef), msg, sizeof(msg_base_pos_ecef_t));
}
void Piksi::_settings_read_resp_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_settings_read_resp), msg, sizeof(msg_settings_read_resp_t));
}
void Piksi::_heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_heartbeat), msg, sizeof(msg_heartbeat_t));
}
void Piksi::_uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_uart_state), msg, sizeof(msg_uart_state_t));
}
void Piksi::_user_data_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    memcpy((u8*)(&_user_data), msg, sizeof(msg_user_data_t));
}