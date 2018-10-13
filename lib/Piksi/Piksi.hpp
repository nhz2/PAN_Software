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
    Piksi(HardwareSerial &serial_port);

    // Standard device functions
    bool setup() override;
    bool is_functional() override;
    void reset() override;
    void disable() override;
    void single_comp_test() override;

    /** \brief Gets GPS time. 
     *  \return GPS time, as nanoseconds since the epoch. **/
    uint64_t get_gps_time();

    /** \brief Gets Dilution of Precision timestamp. 
     *  \return Time-of-week of dilution precision report, in milliseconds. **/
    uint32_t get_dops_tow();
    /** \brief Gets Geometric Dilution of Precision. 
     *  \return Geometric dilution of precision. **/
    uint16_t get_dops_geometric();
    /** \brief Gets Position Dilution of Precision. 
     *  \return Position dilution of precision. **/
    uint16_t get_dops_position();
    /** \brief Gets Time Dilution of Precision. 
     *  \return Time dilution of precision. **/
    uint16_t get_dops_time();
    /** \brief Gets Horizontal Dilution of Precision. 
     *  \return Horizontal dilution of precision. **/
    uint16_t get_dops_horizontal();
    /** \brief Gets Vertical Dilution of Precision. 
     *  \return Vertical dilution of precision. **/
    uint16_t get_dops_vertical();

    /** \brief Gets satellite position in ECEF coordinates.
     *  \param tow A pointer to a storage variable for the time-of-week of the GPS measurement.
     *  \param position A pointer to an array for storing the x,y,z coordinates of the GPS position.
     *  \param accuracy A pointer to a storage variable for the accuracy of the GPS measurement. **/
    void get_pos_ecef(double* tow, double* position[3], double* accuracy);
    /** \brief Gets satellite position in ECEF coordinates relative to base station.
     *  \param tow A pointer to a storage variable for the time-of-week of the GPS measurement.
     *  \param position A pointer to an array for storing the x,y,z coordinates of the GPS position.
     *  \param accuracy A pointer to a storage variable for the accuracy of the GPS measurement. **/
    void get_baseline_ecef(double* tow, double* position[3], double* accuracy);
    /** \brief Gets satellite velocity in ECEF coordinates.
     *  \param tow A pointer to a storage variable for the time-of-week of the GPS measurement.
     *  \param position A pointer to an array for storing the x,y,z coordinates of the GPS position.
     *  \param accuracy A pointer to a storage variable for the accuracy of the GPS measurement. **/
    void get_vel_ecef(double* tow, double* position[3], double* accuracy);
    /** \brief Gets base station position in ECEF coordinates.
     *  \param position A pointer to an array for storing the x,y,z coordinates of the base station. **/
    void get_base_pos_ecef(double* position[3]);

    /** \brief Reads current settings in Piksi RAM.
     *  \return Current settings in Piksi RAM, as a libsbp struct. **/
    char* get_settings_read_resp();

    /** \brief Reads status flags of Piksi (i.e. the "heartbeat").
     *  \return Status flags of Piksi, as a libsbp struct. **/
    uint32_t get_heartbeat();

    /** \brief Reads UART channel A transmission throughput.
     *  \return UART A channel transmission throughput. **/
    float get_uart_a_tx_throughput();
    /** \brief Reads UART channel A reception throughput.
     *  \return UART A channel reception throughput. **/
    float get_uart_a_rx_throughput();
    /** \brief Reads UART channel A CRC error count.
     *  \return UART A channel CRC error count. **/
    uint16_t get_uart_a_crc_error_count();
    /** \brief Reads UART channel A I/O error count.
     *  \return UART A channel I/O error count. **/
    uint16_t get_uart_a_io_error_count();
    /** \brief Reads UART channel A transmission buffer utilization.
     *  \return UART A channel transmission buffer utilization. **/
    uint8_t get_uart_a_tx_buffer_utilization();
    /** \brief Reads UART channel A reception buffer utilization.
     *  \return UART A channel reception buffer utilization. **/
    uint8_t get_uart_a_rx_buffer_utilization();

    /** \brief Reads UART channel B transmission throughput.
     *  \return UART B channel transmission throughput. **/
    float get_uart_b_tx_throughput();
    /** \brief Reads UART channel B reception throughput.
     *  \return UART B channel reception throughput. **/
    float get_uart_b_rx_throughput();
    /** \brief Reads UART channel B CRC error count.
     *  \return UART B channel CRC error count. **/
    uint16_t get_uart_b_crc_error_count();
    /** \brief Reads UART channel B I/O error count.
     *  \return UART B channel I/O error count. **/
    uint16_t get_uart_b_io_error_count();
    /** \brief Reads UART channel B transmission buffer utilization.
     *  \return UART B channel transmission buffer utilization. **/
    uint8_t get_uart_b_tx_buffer_utilization();
    /** \brief Reads UART channel B reception buffer utilization.
     *  \return UART B channel reception buffer utilization. **/
    uint8_t get_uart_b_rx_buffer_utilization();

    /** \brief Reads user data payload.
     *  \return User data, as a string. **/
    char* get_user_data();

    /** \brief Saves the data settings to flash. **/
    void settings_save();
    /** \brief Writes the desired settings to the Piksi's RAM.
     * \param Struct containing setting changes for the Piksi. **/
    void settings_write(const msg_settings_write_t &settings);

    /** \brief Resets Piksi. **/
    void piksi_reset();

    /** \brief Sends custom user data to the Piksi.
     *  \param User data, as an array of (maximally 255) bytes. **/
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
    static void _gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _vel_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _base_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _settings_read_resp_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context);
    static void _user_data_callback(u16 sender_id, u8 len, u8 msg[], void *context);

    // Required writing function by libsbp. See libsbp.c
    HardwareSerial _serial_port;
    static u32 _uart_write(u8 *buff, u32 n, void *context);

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