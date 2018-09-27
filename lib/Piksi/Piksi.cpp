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

void Piksi::_register_all_callbacks() {
    sbp_register_callback(&_sbp_state, SBP_MSG_GPS_TIME, (sbp_msg_callback_t) &Piksi::_gps_time_callback, nullptr, &_gps_time_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_DOPS, (sbp_msg_callback_t) &Piksi::_dops_callback, nullptr, &_dops_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_POS_ECEF, (sbp_msg_callback_t) &Piksi::_pos_ecef_callback, nullptr, &_pos_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASELINE_ECEF, (sbp_msg_callback_t) &Piksi::_baseline_ecef_callback, nullptr, &_baseline_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_VEL_ECEF, (sbp_msg_callback_t) &Piksi::_vel_ecef_callback, nullptr, &_vel_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF, (sbp_msg_callback_t) &Piksi::_base_pos_ecef_callback, nullptr, &_base_pos_ecef_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_SETTINGS_READ_RESP, (sbp_msg_callback_t) &Piksi::_settings_read_resp_callback, nullptr, &_settings_read_resp_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, (sbp_msg_callback_t) &Piksi::_heartbeat_callback, nullptr, &_heartbeat_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_UART_STATE, (sbp_msg_callback_t) &Piksi::_uart_state_callback, nullptr, &_uart_state_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_USER_DATA, (sbp_msg_callback_t) &Piksi::_user_data_callback, nullptr, &_user_data_callback_node);
}

bool Piksi::setup() {
    sbp_state_init(&_sbp_state);

    // Register all necessary callbacks for data reads--specification provided in sbp.c
    _register_all_callbacks();

    return true;
}

bool Piksi::is_functional() { return get_heartbeat().flags == 0; }
void Piksi::reset() { piksi_reset(); }