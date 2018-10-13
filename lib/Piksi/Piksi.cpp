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

// Initialize callback nodes so that C++ doesn't complain about them not being defined.
sbp_msg_callbacks_node_t Piksi::_gps_time_callback_node;
sbp_msg_callbacks_node_t Piksi::_dops_callback_node;
sbp_msg_callbacks_node_t Piksi::_pos_ecef_callback_node;
sbp_msg_callbacks_node_t Piksi::_baseline_ecef_callback_node;
sbp_msg_callbacks_node_t Piksi::_vel_ecef_callback_node;
sbp_msg_callbacks_node_t Piksi::_base_pos_ecef_callback_node;
sbp_msg_callbacks_node_t Piksi::_settings_read_resp_callback_node;
sbp_msg_callbacks_node_t Piksi::_heartbeat_callback_node;
sbp_msg_callbacks_node_t Piksi::_uart_state_callback_node;
sbp_msg_callbacks_node_t Piksi::_user_data_callback_node;

Piksi::Piksi(HardwareSerial &serial_port) {
    _serial_port = serial_port;
}

bool Piksi::setup() {
    sbp_state_init(&_sbp_state);
    sbp_state_set_io_context(&_sbp_state, this);

    sbp_msg_callback_t _gps_time_callback_ptr = &Piksi::_gps_time_callback;
    sbp_msg_callback_t _dops_callback_ptr = &Piksi::_dops_callback;
    sbp_msg_callback_t _pos_ecef_callback_ptr = &Piksi::_pos_ecef_callback;
    sbp_msg_callback_t _baseline_ecef_callback_ptr = &Piksi::_baseline_ecef_callback;
    sbp_msg_callback_t _vel_ecef_callback_ptr = &Piksi::_vel_ecef_callback;
    sbp_msg_callback_t _base_pos_ecef_callback_ptr = &Piksi::_base_pos_ecef_callback;
    sbp_msg_callback_t _settings_read_resp_callback_ptr = &Piksi::_settings_read_resp_callback;
    sbp_msg_callback_t _heartbeat_callback_ptr = &Piksi::_heartbeat_callback;
    sbp_msg_callback_t _uart_state_callback_ptr = &Piksi::_uart_state_callback;
    sbp_msg_callback_t _user_data_callback_ptr = &Piksi::_user_data_callback;

    uint8_t registration_successful = 0;
    // Register all necessary callbacks for data reads--specification provided in sbp.c
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_GPS_TIME, 
        _gps_time_callback_ptr, nullptr, &Piksi::_gps_time_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_DOPS, 
        _dops_callback_ptr, nullptr, &Piksi::_dops_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_POS_ECEF, 
        _pos_ecef_callback_ptr, nullptr, &Piksi::_pos_ecef_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_BASELINE_ECEF, 
        _baseline_ecef_callback_ptr, nullptr, &Piksi::_baseline_ecef_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_VEL_ECEF, 
        _vel_ecef_callback_ptr, nullptr, &Piksi::_vel_ecef_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF, 
        _base_pos_ecef_callback_ptr, nullptr, &Piksi::_base_pos_ecef_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_SETTINGS_READ_RESP, 
        _settings_read_resp_callback_ptr, nullptr, &Piksi::_settings_read_resp_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, 
        _heartbeat_callback_ptr, nullptr, &Piksi::_heartbeat_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_UART_STATE, 
        _uart_state_callback_ptr, nullptr, &Piksi::_uart_state_callback_node);
    registration_successful |= sbp_register_callback(&_sbp_state, SBP_MSG_USER_DATA, 
        _user_data_callback_ptr, nullptr, &Piksi::_user_data_callback_node);

    return (registration_successful == 0);
}

bool Piksi::is_functional() { return get_heartbeat() == 0; }
void Piksi::reset() { piksi_reset(); }

// Cannot disable Piksi, since Piksi is integral to the system. So this function is left unimplemented.
void Piksi::disable() { }

void Piksi::single_comp_test() { /** TODO **/ }

uint64_t Piksi::get_gps_time() { 
    uint32_t wn = _gps_time.wn;
    uint32_t tow = _gps_time.tow;
    uint32_t ns = _gps_time.ns;
    uint32_t ms_in_wk = 7*24*60*60*1000; // Milliseconds per week
    return (wn * ms_in_wk + tow) * 1000 + ns;
}

uint32_t Piksi::get_dops_tow()  { return _dops.tow; }
uint16_t Piksi::get_dops_geometric()  { return _dops.gdop; }
uint16_t Piksi::get_dops_position()  { return _dops.pdop; }
uint16_t Piksi::get_dops_time()  { return _dops.tdop; }
uint16_t Piksi::get_dops_horizontal()  {  return _dops.hdop; }
uint16_t Piksi::get_dops_vertical()  { return _dops.vdop; }

void Piksi::get_pos_ecef(double* tow, double* position[3], double* accuracy) { 
    *tow = _pos_ecef.tow;
    *position[0] = _pos_ecef.x;
    *position[1] = _pos_ecef.y;
    *position[2] = _pos_ecef.z;
    *accuracy = _pos_ecef.accuracy;
}
void Piksi::get_baseline_ecef(double* tow, double* position[3], double* accuracy) { 
    *tow = _pos_ecef.tow;
    *position[0] = _pos_ecef.x;
    *position[1] = _pos_ecef.y;
    *position[2] = _pos_ecef.z;
    *accuracy = _pos_ecef.accuracy;
}
void Piksi::get_vel_ecef(double* tow, double* position[3], double* accuracy) { 
    *tow = _pos_ecef.tow;
    *position[0] = _pos_ecef.x;
    *position[1] = _pos_ecef.y;
    *position[2] = _pos_ecef.z;
    *accuracy = _pos_ecef.accuracy;
}
void Piksi::get_base_pos_ecef(double* position[3]) { 
    *position[0] = _pos_ecef.x;
    *position[1] = _pos_ecef.y;
    *position[2] = _pos_ecef.z;
}

char* Piksi::get_settings_read_resp()  { return _settings_read_resp.setting; }

uint32_t Piksi::get_heartbeat()  { return _heartbeat.flags; }

float Piksi::get_uart_a_tx_throughput() { return _uart_state.uart_a.tx_throughput; }
float Piksi::get_uart_a_rx_throughput()  { return _uart_state.uart_a.rx_throughput; }
uint16_t Piksi::get_uart_a_crc_error_count() { return _uart_state.uart_a.crc_error_count; }
uint16_t Piksi::get_uart_a_io_error_count() { return _uart_state.uart_a.io_error_count; }
uint8_t Piksi::get_uart_a_tx_buffer_utilization() { return _uart_state.uart_a.tx_buffer_level; }
uint8_t Piksi::get_uart_a_rx_buffer_utilization() { return _uart_state.uart_a.rx_buffer_level; }

float Piksi::get_uart_b_tx_throughput() { return _uart_state.uart_b.tx_throughput; }
float Piksi::get_uart_b_rx_throughput()  { return _uart_state.uart_b.rx_throughput; }
uint16_t Piksi::get_uart_b_crc_error_count() { return _uart_state.uart_b.crc_error_count; }
uint16_t Piksi::get_uart_b_io_error_count() { return _uart_state.uart_b.io_error_count; }
uint8_t Piksi::get_uart_b_tx_buffer_utilization() { return _uart_state.uart_b.tx_buffer_level; }
uint8_t Piksi::get_uart_b_rx_buffer_utilization() { return _uart_state.uart_b.rx_buffer_level; }

char* Piksi::get_user_data()  {
    return (char*) _user_data.contents;
}

void Piksi::settings_save()  {
    sbp_send_message(&_sbp_state, SBP_MSG_SETTINGS_SAVE, SBP_SENDER_ID, 
        0, nullptr, &Piksi::_uart_write);
}
void Piksi::settings_write(const msg_settings_write_t &settings)  {
    sbp_send_message(&_sbp_state, SBP_MSG_SETTINGS_WRITE, SBP_SENDER_ID, 
        sizeof(msg_settings_write_t), (uint8_t*) &settings, &Piksi::_uart_write);
}
void Piksi::piksi_reset()  {
    sbp_send_message(&_sbp_state, SBP_MSG_RESET, SBP_SENDER_ID, 
        0, nullptr, &Piksi::_uart_write);
}
void Piksi::send_user_data(const msg_user_data_t &data)  {
    sbp_send_message(&_sbp_state, SBP_MSG_USER_DATA, SBP_SENDER_ID,
        sizeof(msg_user_data_t), (uint8_t*) &data, &Piksi::_uart_write);
}

u32 Piksi::_uart_write(u8 *buff, u32 n, void *context) {
    Piksi* piksi = (Piksi*) context;
    HardwareSerial sp = piksi->_serial_port;
    u32 i;
    for(i = 0; i < n; i++) {
        if (sp.write(buff[i]) == 0) break;
    }
    return i;
}

void Piksi::_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_gps_time)), msg, sizeof(msg_gps_time_t));
}
void Piksi::_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_dops)), msg, sizeof(msg_dops_t));
}
void Piksi::_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_pos_ecef)), msg, sizeof(msg_pos_ecef_t));
}
void Piksi::_baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_baseline_ecef)), msg, sizeof(msg_baseline_ecef_t));
}
void Piksi::_vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_vel_ecef)), msg, sizeof(msg_vel_ecef_t));
}
void Piksi::_base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_base_pos_ecef)), msg, sizeof(msg_base_pos_ecef_t));
}
void Piksi::_settings_read_resp_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_base_pos_ecef)), msg, sizeof(msg_settings_read_resp_t));
}
void Piksi::_heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_heartbeat)), msg, sizeof(msg_heartbeat_t));
}
void Piksi::_uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_uart_state)), msg, sizeof(msg_uart_state_t));
}
void Piksi::_user_data_callback(u16 sender_id, u8 len, u8 msg[], void *context) {
    Piksi* piksi = (Piksi*) context;
    memcpy((u8*)(&(piksi->_user_data)), msg, sizeof(msg_user_data_t));
}