#include "Piksi.h"
Piksi piksi(Serial1);

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {

    static int GPS_week; //  GPS week (number of whole weeks since the first epoch)
    static long GPS_ToW; // GPS time of week (num of seconds since beginning of the week)
    static long GPS_timestamp; // GPS time (num of seconds since first epoch)
    
//Position Solution - absolute geodetic coordinates
    static double GPS_lat; // personal latitude
    static double GPS_lng; // personal longitude
    static double GPS_height; // height 
    static int GPS_pos_flags; //  single point position, float, or fixed mode 

//Velocity ECEF - relative vector distance from base station to rover receiver
    static double GPS_vel_x; //  north
    static double GPS_vel_y; // east
    static double GPS_vel_z; // down
    static int GPS_vel_numSats; //  number of satellites used in soln

//Baseline NED - relative vector distance from base station to rover receiver
    static double GPS_n; //  north
    static double GPS_e; // east
    static double GPS_d; // down
    static int GPS_numSats_shared; //  number of satellites in common (used in soln)
    static int GPS_base_NEDflags; //  flag from the baseline tab

//Baseline ECEF - relative vector distance from base station to rover receiver
    static double GPS_base_x; //  north
    static double GPS_base_y; // east
    static double GPS_base_z; // down
    static int GPS_base_numSats; //  number of satellites in common (used in soln)
    static int GPS_base_flags; //  flag from the baseline tab


//Baseline Heading -  heading pointing from base station to rover relative to True North
    static double GPS_heading; //  heading in mdeg
    static int GPS_head_flags; //  flag from the baseline tab


    bool updated = piksi.updateGPS(GPS_week, GPS_ToW, GPS_timestamp,
            GPS_lat, GPS_lng, GPS_height, GPS_pos_flags,
            GPS_vel_x, GPS_vel_y, GPS_vel_z, GPS_vel_numSats,
            GPS_n, GPS_e, GPS_d, GPS_numSats_shared, GPS_base_NEDflags,
            GPS_base_x, GPS_base_y, GPS_base_z, GPS_base_numSats, GPS_base_flags, 
            GPS_heading, GPS_head_flags);

  if (updated == true)
  {
        Serial.print("GPS Week Num: ");
        Serial.print(GPS_week);
        Serial.print(";  GPS Time of Week: ");
        Serial.print(GPS_ToW);
        Serial.print(";  GPS timestamp: ");
        Serial.println(GPS_timestamp);
        
        Serial.print("Latitude ");
        Serial.print(GPS_lat,7);  
        Serial.print(";  Longitude ");   
        Serial.print(GPS_lng,7);
        Serial.print(";  Height ");   
        Serial.print(GPS_height,7);
        Serial.print(";  Position Soln Flags: ");   
        Serial.println(GPS_pos_flags);

        
        Serial.print("Vel X: ");
        Serial.print(GPS_vel_x);
        Serial.print("; Vel Y: ");
        Serial.print(GPS_vel_y);
        Serial.print("; Vel Z: ");
        Serial.print(GPS_vel_z);
        Serial.print("; Num Sats Vel: ");
        Serial.println(GPS_vel_numSats);
              
        
//        Serial.print("North: ");
//        Serial.print(GPS_n);
//        Serial.print("; East: ");
//        Serial.print(GPS_e);
//        Serial.print("; Down: ");
//        Serial.println(GPS_d);
//        Serial.print("Num of Shared Sats: ");
//        Serial.print(GPS_numSats_shared);
//        Serial.print("; NED Baseline Flags: ");
//        Serial.println(GPS_base_NEDflags);

        Serial.print("ECEF X: ");
        Serial.print(GPS_base_x);
        Serial.print("; ECEF Y: ");
        Serial.print(GPS_base_y);
        Serial.print("; ECEF Z: ");
        Serial.println(GPS_base_z);
        Serial.print("Num of Shared Sats: ");
        Serial.print(GPS_base_numSats);
        Serial.print("; Baseline Flags: ");
        Serial.println(GPS_base_flags);
        
        Serial.print("Baseline Heading:");
        Serial.print(GPS_heading);
        Serial.print("Heading Flags:");
        Serial.println(GPS_head_flags);
        Serial.println();
  }
  
}
