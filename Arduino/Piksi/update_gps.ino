/*!
* \file       update_gps.ino
*
*
* \date       2018/03/20
*
* \author     
* 
* \brief      

\details
- 
*/




#include <SPI.h>
#include <Wire.h>
#include <Time.h>


const unsigned int MAX_INPUT = 2000;

void setup()
{
    HardwareSerial mySerial = Serial1;

    Serial.begin(115200);                                // serial to pc
    mySerial.begin (115200);                              // serial1 to GPS
    Serial.println("Setup complete.");

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); 
}


// Definition of SBP Protocol here: https://github.com/swift-nav/libswiftnav/raw/master/docs/sbp.pdf

#define		EndofMessage	    0x55

//GPS Time
#define    Byte0Time         0x00
#define   Byte1Time         0x01
#define     LengthTime              18

//LatLonAlt
#define 	Byte0LatLon 	    0x01
#define 	Byte1LatLon				0x02
#define     LengthLatLon            41

//Velocity ECEF
#define   Byte0Vel         0x04
#define   Byte1Vel         0x02
#define     LengthVel               27

//Baseline NED
#define   Byte0NED          0x03
#define   Byte1NED          0x02
#define     LengthNED               29

//Baseline ECEF
#define   Byte0ECEF         0x02
#define   Byte1ECEF         0x02
#define     LengthECEF              27

//Baseline Heading
#define   Byte0Head         0x07
#define   Byte1Head         0x02
#define     LengthHead              17


void loop()
{

  
    int newdata=0;

//GPS Time
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

    

    while (mySerial.available () > 0)
    {
        processIncomingByte (mySerial.read (), GPS_week, GPS_ToW, GPS_timestamp,
            GPS_lat, GPS_lng, GPS_height, GPS_pos_flags,
            GPS_vel_x, GPS_vel_y, GPS_vel_z, GPS_vel_numSats,
            GPS_n, GPS_e, GPS_d, GPS_numSats_shared, GPS_base_NEDflags,
            GPS_base_x, GPS_base_y, GPS_base_z, GPS_base_numSats, GPS_base_flags, 
            GPS_heading, GPS_head_flags, newdata);
    }



    if (newdata==true)
    {
        mySerial.flush();

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







void processIncomingByte (const byte inByte, int& GPS_week, long& GPS_ToW, long& GPS_timestamp, 
    double& GPS_lat, double& GPS_lng, double& GPS_height, int& GPS_pos_flags,
    double& GPS_vel_x, double& GPS_vel_y, double& GPS_vel_z, int& GPS_vel_numSats, 
    double& GPS_n, double& GPS_e, double& GPS_d, int& GPS_numSats_shared, int& GPS_base_NEDflags,
    double& GPS_base_x, double& GPS_base_y, double& GPS_base_z, int& GPS_base_numSats, int& GPS_base_flags,
    double& GPS_heading, int& GPS_head_flags, int& newdata)
{
// Serial.println(inByte , HEX);

    static byte input_msg [MAX_INPUT];
    static unsigned int input_pos = 0;             

    static int newdataTime=0;
    static int newdataNED=0;
    static int newdataECEF=0;
    static int newdataVel=0;
    static int newdataLatLon=0;
    static int newdataHead=0;

    if (inByte == EndofMessage)
    {

        /*
        //print serial data from Piksi for debugging
        for( int i = 0 ; i < input_pos ; i++)
        {
                Serial.print(input_msg [i],HEX);       // wenn hex 55 print gesammter puffer [input_msg)
                Serial.print(" ");
        }
         Serial.println();
         */



        if (input_pos >=LengthTime || input_pos >=LengthNED || input_pos >=LengthLatLon)       //purpose
        {
            // Time Message
            if((input_msg[0] == Byte0Time) && (input_msg[1] == Byte1Time)&& input_pos == LengthTime)
            {
                msg_analyse_time (input_msg, GPS_week, GPS_ToW, GPS_timestamp);
                newdataTime=1;
                input_pos = 0;
            }

            
            // LatLon Positon Message
            else if((input_msg[0] == Byte0LatLon) && (input_msg[1] == Byte1LatLon)&& input_pos == LengthLatLon)
            {
                msg_analyse_latlon (input_msg, GPS_lat, GPS_lng, GPS_height, GPS_pos_flags);
                newdataLatLon=1;
                input_pos = 0;
            }

            // Vel ECEF Message
            else if((input_msg[0] == Byte0Vel) && (input_msg[1] == Byte1Vel) && input_pos == LengthVel)
            {
                msg_analyse_vel (input_msg, GPS_vel_x, GPS_vel_y, GPS_vel_z, GPS_vel_numSats);
                newdataVel=1;
                input_pos = 0;  

            }

            // NED Baseline Message
            else if((input_msg[0] == Byte0NED) && (input_msg[1] == Byte1NED) && input_pos == LengthNED)
            {
                msg_analyse_ned (input_msg, GPS_n, GPS_e, GPS_d, GPS_numSats_shared, GPS_base_NEDflags);
                newdataNED=1;
                input_pos = 0;
            }


            // ECEF Baseline Message
            else if((input_msg[0] == Byte0ECEF) && (input_msg[1] == Byte1ECEF) && input_pos == LengthECEF)
            {
                msg_analyse_ecef (input_msg, GPS_base_x, GPS_base_y, GPS_base_z, GPS_base_numSats, GPS_base_flags);
                newdataECEF=1;
                input_pos = 0;   
                
            }

            // Heading Message
            else if((input_msg[0] == Byte0Head) && (input_msg[1] == Byte1Head) && input_pos == LengthHead)
            {
                msg_analyse_head (input_msg, GPS_heading, GPS_head_flags);
                newdataHead=1;
                input_pos = 0;   
                
            }

            else {}


        }
        // after message is parsed, set back to beginning
        input_pos = 0;

        if (newdataTime == true && newdataLatLon==true && newdataNED ==true && newdataECEF ==true && newdataVel ==true)
        {                                                         //purpose
            newdata=true;
            newdataTime =false;
            newdataLatLon =false;
            newdataVel =false;
            newdataNED =false;
            newdataECEF =false;
            newdataHead =false;

        }

        
    }

    // if no complete message in buffer, read out again and fill up input_msg
    else
    {

        if (input_pos < (MAX_INPUT - 1))
        {
            input_msg [input_pos] = inByte;
            input_pos = input_pos + 1;
        }
    }

}


void msg_analyse_time (byte byte_msg[40],int& GPS_week, long& GPS_ToW, long& GPS_timestamp)
{
    int nrofweek=0;
    int timeofweek=0;

    nrofweek = Bytes2Intu16 ( byte_msg[6], byte_msg[5]);
    timeofweek = Bytes2Intu32 (byte_msg[10], byte_msg[9], byte_msg[8], byte_msg[7]);
    
    GPS_week = nrofweek;
    GPS_ToW = timeofweek;
    GPS_timestamp = timeofweek+nrofweek*7*24*3600;
    
}

void msg_analyse_latlon (byte byte_msg[41],double& GPS_lat, double& GPS_lng, double& GPS_height, int& GPS_pos_flags)
{

    GPS_lat = Bytes2Double (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13],byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    GPS_lng = Bytes2Double (byte_msg[24], byte_msg[23], byte_msg[22], byte_msg[21],byte_msg[20], byte_msg[19], byte_msg[18], byte_msg[17]);

    GPS_height = Bytes2Double (byte_msg[32], byte_msg[31], byte_msg[30], byte_msg[29],byte_msg[28], byte_msg[27], byte_msg[26], byte_msg[25]);
    GPS_pos_flags = byte_msg[38];
}


void msg_analyse_vel (byte byte_msg[40], double& GPS_vel_x, double& GPS_vel_y, double& GPS_vel_z, int& GPS_vel_numSats)
{
    GPS_vel_x = Bytes2Intu32 (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    GPS_vel_y  = Bytes2Intu32 (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);
    GPS_vel_z = Bytes2Intu32 (byte_msg[20], byte_msg[19], byte_msg[18], byte_msg[17]);
    GPS_vel_numSats = byte_msg[23];

}


void msg_analyse_ned (byte byte_msg[40], double& GPS_n, double& GPS_e, double& GPS_d, int& GPS_numSats_shared, int& GPS_base_NEDflags)
{
    GPS_n = Bytes2Intu32 (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    GPS_e  = Bytes2Intu32 (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);
    GPS_d = Bytes2Intu32 (byte_msg[20], byte_msg[19], byte_msg[18], byte_msg[17]);
    GPS_numSats_shared = byte_msg[25];
    GPS_base_NEDflags = byte_msg[26];
}


void msg_analyse_ecef (byte byte_msg[40], double& GPS_base_x, double& GPS_base_y, double& GPS_base_z, int& GPS_base_numSats, int& GPS_base_flags)
{
    GPS_base_x = Bytes2Intu32 (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    GPS_base_y  = Bytes2Intu32 (byte_msg[16], byte_msg[15], byte_msg[14], byte_msg[13]);
    GPS_base_z = Bytes2Intu32 (byte_msg[20], byte_msg[19], byte_msg[18], byte_msg[17]);
    GPS_base_numSats = byte_msg[23];
    GPS_base_flags = byte_msg[24];

}


void msg_analyse_head (byte byte_msg[40], double& GPS_heading, int& GPS_head_flags)
{
    GPS_heading = Bytes2Intu32 (byte_msg[12], byte_msg[11], byte_msg[10], byte_msg[9]);
    GPS_head_flags = byte_msg[11];

}



int Bytes2Intu32 (int b4, int b3, int b2, int b1)
{
    int result=0;
    result=(b4 << 24) | (b3<<16) | (b2<<8) | b1;
    return result;
}

// convert 2 bytes to int
int Bytes2Intu16 (int b2, int b1)
{
    int result=0;
    result=(b2<<8) | b1;
    return result;
}


// convert 8 bytes to double
double Bytes2Double (int b8,int b7,int b6,int b5,int b4, int b3, int b2, int b1)
{

    union u_tag
    {
        byte b[8];
        double fval;
    } u;

    u.b[0] = b1;
    u.b[1] = b2;
    u.b[2] = b3;
    u.b[3] = b4;
    u.b[4] = b5;
    u.b[5] = b6;
    u.b[6] = b7;
    u.b[7] = b8;

    return u.fval;
}


