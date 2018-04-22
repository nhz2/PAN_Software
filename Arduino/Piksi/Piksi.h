#ifndef Piksi_h
#define Piksi_h

#include <Arduino.h>


class Piksi
{
  public:
    Piksi(HardwareSerial piksiSerial);
    bool updateGPS(int& GPS_week, long& GPS_ToW, long& GPS_timestamp, 
      double& GPS_lat, double& GPS_lng, double& GPS_height, int& GPS_pos_flags,
      double& GPS_vel_x, double& GPS_vel_y, double& GPS_vel_z, int& GPS_vel_numSats, 
      double& GPS_n, double& GPS_e, double& GPS_d, int& GPS_numSats_shared, int& GPS_base_NEDflags,
      double& GPS_base_x, double& GPS_base_y, double& GPS_base_z, int& GPS_base_numSats, int& GPS_base_flags,
      double& GPS_heading, int& GPS_head_flags);

  private:
    HardwareSerial mySerial;

    void processIncomingByte (const byte inByte, int& GPS_week, long& GPS_ToW, long& GPS_timestamp, 
      double& GPS_lat, double& GPS_lng, double& GPS_height, int& GPS_pos_flags,
      double& GPS_vel_x, double& GPS_vel_y, double& GPS_vel_z, int& GPS_vel_numSats, 
      double& GPS_n, double& GPS_e, double& GPS_d, int& GPS_numSats_shared, int& GPS_base_NEDflags,
      double& GPS_base_x, double& GPS_base_y, double& GPS_base_z, int& GPS_base_numSats, int& GPS_base_flags,
      double& GPS_heading, int& GPS_head_flags, int& newdata);


    void msg_analyse_time (byte byte_msg[40],int& GPS_week, long& GPS_ToW, long& GPS_timestamp);
    void msg_analyse_latlon (byte byte_msg[41],double& GPS_lat, double& GPS_lng, double& GPS_height, int& GPS_pos_flags);
    void msg_analyse_vel (byte byte_msg[40], double& GPS_vel_x, double& GPS_vel_y, double& GPS_vel_z, int& GPS_vel_numSats);
    void msg_analyse_ned (byte byte_msg[40], double& GPS_n, double& GPS_e, double& GPS_d, int& GPS_numSats_shared, int& GPS_base_NEDflags);
    void msg_analyse_ecef (byte byte_msg[40], double& GPS_base_x, double& GPS_base_y, double& GPS_base_z, int& GPS_base_numSats, int& GPS_base_flags); 
    void msg_analyse_head (byte byte_msg[40], double& GPS_heading, int& GPS_head_flags);
    
    int Bytes2Intu32 (int b4, int b3, int b2, int b1);
    int Bytes2Intu16 (int b2, int b1);
    double Bytes2Double (int b8,int b7,int b6,int b5,int b4, int b3, int b2, int b1);
    
};
#endif
