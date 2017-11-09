
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <EEPROM.h>

//State Machine Definition
#define DORMANT_CRUISE  1
#define INITALIZATION   2
#define NORMAL_OPS      3
#define ECLIPSE         4
#define SAFE_HOLD       5
#define DOCKED          6

bool WireConnected = true;
unsigned long LastTimeTime = 0;
unsigned int TimeTime = 4000;

////Constant Initialization
unsigned long cruiseEnd = 45 * 60 * 1000;
unsigned long ledLastTime = millis();
unsigned long cycle = 0;
bool ledState = LOW;
unsigned long manualTimeout = 10 * 1000;
unsigned int ADCSResets = 0;
unsigned long deployTimeOut = 30 * 1000;

//State Machine Transition Flags and Times
unsigned long eclipseEntry;
unsigned long lowPowerEntry;
unsigned long normOpEntry;
unsigned long initEntry;
unsigned long forceExitEclipseTime = 60 * 60 * 1000;
unsigned long lastAccelTime;

//Threshold Values
float LV_Threshold = 6.6; //Volts
float HV_Threshold = 7.0; //Volts
int8_t LT_Threshold = -10; //C
uint8_t HT_Threshold = 60; //C

//Battery Test

unsigned long LastBattCheck = 0;
uint16_t BattCheckTime = 8000;
unsigned long LastSolarCheck = 0;
unsigned long lastRadioCheck = 0;
unsigned long RadioCheckTime = 6000;
unsigned long DLTime = (2 * 60 * 1000); //2 min
unsigned long lastDLTime = 0;

//IMU and Sensor Test
//TODO IMU Object
LSM6 imu;
LIS3MDL mag;
int SensorDwell = 10; //100; //Averaging Time Non-BLOCKING!
unsigned long lastSensorTime = 0;
int DataRecords = 0;

//ADCS Communication Test
unsigned long recentADCSCom = 0;
unsigned long lastADCSComTime = 0;
unsigned long lastADCSComAttempt = 0;
unsigned int ADCSComTime = 2001;
unsigned long ADCSResetTimeOut = 30 * 1000;

//ADCS Test
unsigned long LastSpinCheckT = 0;
unsigned long SpinCheckTime = 7010;
float OmegaThreshold = 30; //Degrees per second

//Thermal Test
long LastThermalCheck = 0;
int ThermalCheck = 6100;

//Serial Command Test
int popTime = 4000;
unsigned long lastPopTime = 0;

//RockBlock Test
bool testRDL = false;

//Commanded Action Flags
bool commandedSC = false;
bool commandedDL = false;

//Pinout Numbers
#define ADCSReset         2
#define DockingForward    3
#define DockingBackward   4
#define DockingEnable     5
#define TempMux1          6
#define TempMux2         11
#define Valve1           13
#define Valve2           14
#define Valve3           15
#define Valve4           18
#define PressSens        A5
#define TempMuxRead      A7
#define Valve5           22

//EEPROM Addresses
#define RebootsL 0x01
#define Thrust1L 0x02
#define Thrust2L 0x03
#define Thrust3L 0x04
#define Thrust4L 0x05

//TODO System Time?
// EEPROM.put() for Structs
// EEPROM.length()


bool DA_Initialize;

class floatTuple
{
  public:
    float x;
    float y;
    float z;

    floatTuple(float a, float b, float c) {
      x = a;
      y = b;
      z = c;
    }
    void print() {
      Serial.print(x); Serial.print(F(" "));
      Serial.print(y); Serial.print(F(" "));
      Serial.print(z); Serial.println(F(" "));
    }
};

void print_binary(int v, int num_places) {
  int mask = 0, n;
  for (n = 1; n <= num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask;  // truncate v to specified number of places
  while (num_places) {
    if (v & (0x0001 << num_places - 1)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    --num_places;
  }
}

void printArray(uint8_t arr[], int s) {
  for (int i = 0; i < s; i++) {
    print_binary(arr[i], 8);
    Serial.print(" ");
  }
  Serial.println("");
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class GPSData {
  public:

    int GPS_year; //  GPS year
    int GPS_week; //  GPS week (number of whole weeks since the first epoch)
    int GPS_day; // GPS day
    int GPS_month; // GPS month
    int GPS_hour; //  GPS hour
    int GPS_minute; //  GPS minute
    int GPS_sec; // GPS seconds
    int GPS_deSecond; //  decimal part of second (milliseconds)
    long GPS_ToW; // GPS time of week (num of seconds since beginning of the week)
    int GPS_numSats; // number of satellites Piksi is tracking
    float GPS_lat; // personal latitude
    float GPS_lng; // personal longitude
    float GPS_alt; // altitude 
    int GPS_flags; // current Piksi mode
    int GPS_mode; //  single point position, float, or fixed mode 
    float GPS_velN; //  Piksi's Northern velocity
    float GPS_velE; //  Piksi's Eastern velocity
    float GPS_velD; //  Piksi's vertical "down" velocity
    float GPS_PDOP; //  position error (Position Dilution of Precision)
    float GPS_GDOP; //  position error (Geometric Dilution of Precision)
    float GPS_TDOP; //  position error (Time Dilution of Precision)
    float GPS_HDOP; //  position error (Horizontal Dilution of Precision)
    float GPS_VDOP; //  position error (Vertical Dilution of Precision)

    float GPS_n; //  north
    float GPS_e; // east
    float GPS_d; // down
    float GPS_dist; //  distance
    int GPS_numSats_shared; //  number of satellites in common
    int GPS_flags_base; //  flag from the baseline tab
    int GPS_mode_base; // mode from the baseline tab
    int GPS_iarNum; //  IAR number

    int GPS_prn; //  PRN
    float GPS_pseudoRange; // Pseudorange
    float GPS_carrPhase; // Carrier Phase 
    float GPS_C2N; // C/N0 
    float GPS_dop; // Doppler

    GPSData() {
      GPS_year = 0;
    }

    bool updateData() {
      //TODO Fetch Data from Piksi
    }
};

class masterStatus {
    //Class to hold entire State of Spacecraft Operation except timers
  public:


    bool hardwareAvTable[11];//Hardware Avaliability Table
    //[Imu, SX+,SX-,SY+, SY-, SZ+, SZ-,Temp,DoorSense,LightSense,ADCS]
    //Fix PopCommand Av Swap Limit if changed

    // change final string to binary. get bytes. find upper and lower limits. round floats and set value for maxes (like MAX)
    int State;
    int NextState;

    //IMU Data Variables
    float Mag[3]; // max 0.65 gauss min -0.65 gauss
    float Gyro[3];  // max 245 dps min -245 dps
    float Accel[3]; //?
    float MagAcc[3]; //Accumulator
    float GyroAcc[3]; //Accumulator
    float AccelAcc[3]; //Accumulator
    float GyroZero[3]; //Gyro Origin
    float Temp[4];
    float TempAcc[4]; // TODO Range

    //Sensor Variables
    float Battery; // TODO Range
    float PhotoTrans[15];

    //ADCS State Variables
    float RWA1_RotorSpeed;//  Rotor speed of CMG 1
    float RWA1_RotorTorque;//  Rotor torque of CMG 1
    float RWA2_RotorSpeed;// Rotor speed of CMG 1
    float RWA2_RotorTorque;//  Rotor torque of CMG 1
    float RWA3_RotorSpeed;// Rotor speed of CMG 1
    float RWA3_RotorTorque;//  Rotor torque of CMG 1

    float SunX;//  Sun Sensor x component {Ch 6 or 9}
    float SunY;//  Sun Sensor y component {Ch 6 or 9}
    float SunZ;//  Sun Sensor z component {Ch 6 or 9}

    int MagTor_state;//1,2,3 state for magnetorquer desaturation, demagnize, normal
    int TorqX_PWM;
    int TorqY_PWM;
    int TorqZ_PWM;


    float AttitudeX;// attitude vector (actual) from ADCS computer
    float AttitudeY;// attitude vector (actual) from ADCS computer
    float AttitudeZ;// attitude vector (actual) from ADCS computer
    float AttitudeDesiredX;//  desired attitude vector (actual) from ADCS computer
    float AttitudeDesiredY;//  desired attitude vector (actual) from ADCS computer
    float AttitudeDesiredZ;//  desired attitude vector (actual) from ADCS computer
    int ADCS_state;//  state of the entire ADCS system
    float Hall_Mag;//  reading from hall sensors for magnitorquers (magnetic field)
    int Desat_count;// runnning counter of the number times we go into desaturation 

    bool ADCS_on;// ADCS on ? only during checkout


    //Propulsion Variables
    int ThrustFiring; //0-14 Last thrust firing combination
    long ThrustStart; //Millis Time for thruster to fire
    long ThurstDuration; //Duration of firing in millis
    int Thrust1Fire; //T1 firing counter
    int Thrust2Fire; //T2 firing counter
    int Thrust3Fire; //T3 firing counter
    int Thrust4Fire; //T4 firing counter

    float PresBeforeFire; //Tank Psi Before fire
    float PresAfterFire; //Tank Psi After fire
    float PresDownlink; //Tank Psi Before DL

    float Temp1Before; //temperature sensor1 before thruster fired
    float Temp2Before; //temperature sensor2 before thruster fired
    float Temp1AfterFire; //temperature sensor1 after thrusters fired
    float Temp2AfterFire; //temperature sensor2 after thruster fired
    float Temp1Downlink; //temperature sensor1 before downlink
    float Temp2Downlink; //temperature sensor2 before downlink

    //Docking Variables
    int MAGstate; // aligned or unaligned
    bool MAGflip; // magnets have flipped 
    int MAGcount; // time magnets changed state

    //GPS Data Holder
    GPSData GPS;

    //Power System Variables
    float voltage;
    float input_current;
    float input_power;
    float output_current;
    float output_power;
    float efficiency;

    int header1_number;
    float header1_current;
    bool header1_enabled;
    int header2_number;
    float header2_current;
    bool header2_enabled;
    int header3_number;
    float header3_current;
    bool header3_enabled;
    int header4_number;
    float header4_current;
    bool header4_enabled;
    int header5_number;
    float header5_current;
    bool header5_enabled;
    int header6_number;
    float header6_current;
    bool header6_enabled;
    int header7_number;
    float header7_current;
    bool header7_enabled;
    int header8_number;
    float header8_current;
    bool header8_enabled;

    float photo1_voltage;
    float photo1_current;
    float photo1_power;
    float photo2_voltage;
    float photo2_current;
    float photo2_power;
    float photo3_voltage;
    float photo3_current;
    float photo3_power;

    //Radio Message Variables



    masterStatus() {
      //Constructor
      State = 4; //Normal Ops //TODO
      NextState = State;

      //bool hardwareAvTable[10] = {true}; // Hardware Avaliability Table
      //TODO

      //IMU
      Gyro[3] = {0};
      Mag[3] = {0};
      GyroAcc[3] = {0};
      MagAcc[3] = {0};
      GyroZero[3] = {0};
      Temp[4] = {0};
      TempAcc[4] = {0};

      //Sensors
      Battery = 3.8; //
      PhotoTrans[15] = {0};

      //ADCS
      TorqX_PWM = 0;
      TorqY_PWM = 0;
      TorqZ_PWM = 0;
    }


    String toString() {
      //Produces Output in ASCII for Printing and Downlink
      //TODO
      return "TODO";
    }

    String OutputString() {
      //round floats first
      //(round(),2)
      //constrain
      //getbytes for loop
      String OutputString = "";
      //TODO Telemetry
      return OutputString;
    }
};
masterStatus MSH; //Declare MSH


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////// Sensor Functions ///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//IMU Code
void getIMUData() {
  //Fetches IMU and Mag Data for Time averaging to reduce noise
  mag.read();
  imu.read();
  MSH.MagAcc[0] += mag.m.x; MSH.MagAcc[1] += mag.m.y; MSH.MagAcc[2] += mag.m.z;
  MSH.GyroAcc[0] += imu.g.x; MSH.GyroAcc[1] += imu.g.y; MSH.GyroAcc[2] += imu.g.z;
  MSH.AccelAcc[0] += imu.a.x; MSH.AccelAcc[1] += imu.a.y; MSH.AccelAcc[2] += imu.a.z;
}

void getTempSensors() {
  digitalWrite(TempMux1, LOW);
  digitalWrite(TempMux2, LOW);
  delayMicroseconds(5);
  MSH.TempAcc[0] += analogRead(TempMuxRead) * (3.3 / 1024.0) * 1000; //mV //TODO Degrees C
  digitalWrite(TempMux2, HIGH);
  delayMicroseconds(5);
  MSH.TempAcc[1] += analogRead(TempMuxRead) * (3.3 / 1024.0) * 1000; //mV
  digitalWrite(TempMux1, HIGH);
  digitalWrite(TempMux2, LOW);
  delayMicroseconds(5);
  MSH.TempAcc[2] += analogRead(TempMuxRead) * (3.3 / 1024.0) * 1000; //mV
  digitalWrite(TempMux1, HIGH);
  digitalWrite(TempMux2, HIGH);
  delayMicroseconds(5);
  MSH.TempAcc[3] += analogRead(TempMuxRead) * (3.3 / 1024.0) * 1000; //mV
  digitalWrite(TempMux1, LOW);
  digitalWrite(TempMux2, LOW);
}

void SensorDataCollect(int type = 0) { //TODO <-what is the todo for?
  //Collect Sensor Data and Average it if sufficient time has passed
  getIMUData();
  DataRecords++;
  if (millis() - lastSensorTime > SensorDwell) { //SensorDwell ~10ms
    //Add Any Averaging Data

    MSH.Gyro[0] = MSH.GyroAcc[0] / ((float)DataRecords);
    MSH.Gyro[1] = MSH.GyroAcc[1] / ((float)DataRecords);
    MSH.Gyro[2] = MSH.GyroAcc[2] / ((float)DataRecords);
    MSH.Mag[0] = MSH.MagAcc[0] / ((float)DataRecords);
    MSH.Mag[1] = MSH.MagAcc[1] / ((float)DataRecords);
    MSH.Mag[2] = MSH.MagAcc[2] / ((float)DataRecords);

    for (int i = 0; i < 4; i++) {
      MSH.Temp[i] = MSH.TempAcc[i] / ((float)DataRecords);
    }

    MSH.TempAcc[4] = {0};
    MSH.GyroAcc[0] = 0; MSH.GyroAcc[1] = 0; MSH.GyroAcc[2] = 0;
    MSH.MagAcc[0] = 0; MSH.MagAcc[1] = 0; MSH.MagAcc[2] = 0;
    lastSensorTime = millis();
    DataRecords = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////  Command Parsing     ////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class commandBuffer {
    //Class which holds a stack of currently waiting commands which have not been processed
  public:
    int commandStack[200][2];
    int openSpot;
    commandBuffer() {
      commandStack[200][2] = { -1};
      openSpot = 0;
    }
    void print() {
      //Serial formatting and Serial output
      int i = 0;
      Serial.print(F("cBuf = ["));
      int endT = millis() + manualTimeout;
      while (i < 200 && millis() < endT) {
        if (commandStack[i][0] == -1 && commandStack[i][1] == -1) {
          break;
        }
        Serial.print(commandStack[i][0]);
        Serial.print(F(":"));
        Serial.print(commandStack[i][1]);
        Serial.print(F("|"));
        i++;
      }
      Serial.println(F("]"));
    }
};
commandBuffer cBuf;



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Helper Functions

void initalizePinOut() {
  //Setup Master Pinout
  pinMode(ADCSReset, OUTPUT);
  pinMode(DockingForward, OUTPUT);
  pinMode(DockingBackward, OUTPUT);
  pinMode(DockingEnable, OUTPUT);
  pinMode(TempMux1, OUTPUT);
  pinMode(TempMux2, OUTPUT);
  pinMode(Valve1, OUTPUT);
  pinMode(Valve2, OUTPUT);
  pinMode(Valve3, OUTPUT);
  pinMode(Valve4, OUTPUT);
  pinMode(PressSens, INPUT);
  pinMode(TempMuxRead, INPUT);
  pinMode(Valve5, OUTPUT);
}

extern "C" char *sbrk(int i);
int freeRam () {
  //Determine Remaining RAM on Master
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

volatile bool stall = true;
void waitForInterrupt() {
  stall = false;
  //noInterrupts();
}

float roundDecimal(float num, int places) {
  int roundedNum = round(pow(10, places) * num);
  return roundedNum / ((float)(pow(10, places)));
}

String chop(float num, int p) {
  String s = String(num);
  if (p == 0) {
    return s.substring(0, s.indexOf('.'));
  }
  return s.substring(0, s.indexOf('.') + p + 1);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////Parser Functions

void buildBuffer(String com) {
  //Check if incoming String <com> is valid set of commands and add it to the CommandBuffer
  int commandData;
  int commandType;
  String comRemaining = com;
  bool l = true;
  while (l) {
    commandType = (com.substring(0, com.indexOf(","))).toInt();
    commandData = (com.substring(com.indexOf(",") + 1, com.indexOf("!"))).toInt();
    cBuf.commandStack[cBuf.openSpot][0] = commandType;
    cBuf.commandStack[cBuf.openSpot][1] = commandData;
    if (com.indexOf("!") == com.length() - 1) {
      l = false;
      //Serial.println(F("Finished Adding Commands"));
    } else {
      com = com.substring(com.indexOf("!") + 1);
    }
    cBuf.openSpot++;
  }
}

boolean isInputValid(String input) {
  //Check if incoming command string <input> is valid
  int lastPunc = 0; //1 if ",", 2 if "!", 0 Otherwise
  bool valid = true;
  int q = 0;
  int l = input.length();
  int endT = manualTimeout + millis();
  while (q < l) {
    char currentChar = input[q];
    q++;

    if (millis() > endT) {
      valid = false;
      break;
    }

    if (isPunct(currentChar)) {
      if (currentChar == (',')) {
        //Check if last was a period
        //Serial.println("Comma Found");
        if (lastPunc == 0 || lastPunc == 2) {
          //Serial.println("Comma OK");
          lastPunc = 1;
        } else {
          //Serial.println("2 Commas");
          valid = false;
          break;
        }
      } else if (currentChar == ('!')) {
        if (input[q - 2] == ',') {
          //Serial.println("No Second Command Number");
          valid = false;
          break;
        }
        //Serial.println("Excl Found");
        if (lastPunc == 1) {
          //Serial.println("Period ok");
          lastPunc = 2;
        } else {
          //Serial.println("2 Excl or No prior comma");
          valid = false;
          break;
        }
      } else if (currentChar == ('-')) {
        //Serial.println("Hypen Found");
        if (input[q - 2] == ',') { //q incremented after value capture
          //Serial.println("Negative Sign ok");
        } else {
          //Serial.println("Hyphen in wrong place");
          valid = false;
          break;
        }
      } else {
        //Serial.println("Invalid Punc");
        valid = false;
        break;
      }
    } else if (isAlpha(currentChar)) {
      //Serial.println("Alpha");
      valid = false;
      break;
    } else if (isSpace(currentChar)) {
      //Serial.println("Space");
      valid = false;
      break;
    }

    //Detect no ending exclamation point
    if (q == input.length() - 1) {
      if (input[q] != '!') {
        //Serial.println("No Ending");
        valid = false;
        break;
      }
    }
    //Null Character in the middle
    if (currentChar == '\0' && q != input.length() - 1) {
      valid = false;
      break;
    }
  }
  return valid;
}

void popCommands() {
  //Process all the Incoming Commands
  long start = millis();
  while (cBuf.openSpot > 0 && millis() - start < manualTimeout) {
    if (cBuf.openSpot > 0) {
      //Serial.println (cBuf.openSpot - 1);
      int currentCommand[2] = {cBuf.commandStack[cBuf.openSpot - 1][0], cBuf.commandStack[cBuf.openSpot - 1][1]};
      cBuf.commandStack[cBuf.openSpot - 1][0] = -1;
      cBuf.commandStack[cBuf.openSpot - 1][1] = -1;
      cBuf.openSpot --;

      //Supported Commands
      switch (currentCommand[0]) {
        case (93): //Set Manual Function Timeout (millis)
          if (currentCommand[1] >= 2000) {
            manualTimeout = (currentCommand[1]);
            Serial.println(("\nManual Function Timeout set to ") + String(currentCommand[1]) + (" ms"));
          }
          break;

        //ADCS Returned Values
        case (804): //Actual Attitude Yaw
          MSH.AttitudeX = currentCommand[1];
          break;
        case (805): //Actual Attitude Pitch
          MSH.AttitudeY = currentCommand[1];
          break;
        case (806): //Actual Attitude Roll
          MSH.AttitudeZ = currentCommand[1];
          break;
        case (807): //RWA X Speed
          MSH.RWA1_RotorSpeed = currentCommand[1];
          break;
        case (808): //RWA Y Speed
          MSH.RWA2_RotorSpeed = currentCommand[1];
          break;
        case (809): //RWA Z Speed
          MSH.RWA3_RotorSpeed = currentCommand[1];
          break;
        case (810): //RWA X Speed
          MSH.TorqX_PWM = currentCommand[1];
          break;
        case (811): //RWA Y Speed
          MSH.TorqY_PWM = currentCommand[1];
          break;
        case (812): //RWA Z Speed
          MSH.TorqZ_PWM = currentCommand[1];
          break;


        case (813): //Phototransistor mV
          MSH.PhotoTrans[0] = currentCommand[1];
          break;
        case (814): //Phototransistor mV
          MSH.PhotoTrans[1] = currentCommand[1];
          break;
        case (815): //Phototransistor mV
          MSH.PhotoTrans[2] = currentCommand[1];
          break;
        case (816): //Phototransistor mV
          MSH.PhotoTrans[3] = currentCommand[1];
          break;
        case (817): //Phototransistor mV
          MSH.PhotoTrans[4] = currentCommand[1];
          break;
        case (818): //Phototransistor mV
          MSH.PhotoTrans[5] = currentCommand[1];
          break;
        case (819): //Phototransistor mV
          MSH.PhotoTrans[6] = currentCommand[1];
          break;
        case (820): //Phototransistor mV
          MSH.PhotoTrans[7] = currentCommand[1];
          break;
        case (821): //Phototransistor mV
          MSH.PhotoTrans[8] = currentCommand[1];
          break;
        case (822): //Phototransistor mV
          MSH.PhotoTrans[9] = currentCommand[1];
          break;
        case (823): //Phototransistor mV
          MSH.PhotoTrans[10] = currentCommand[1];
          break;
        case (824): //Phototransistor mV
          MSH.PhotoTrans[11] = currentCommand[1];
          break;
        case (825): //Phototransistor mV
          MSH.PhotoTrans[12] = currentCommand[1];
          break;
        case (826): //Phototransistor mV
          MSH.PhotoTrans[13] = currentCommand[1];
          break;
        case (827): //Phototransistor mV
          MSH.PhotoTrans[14] = currentCommand[1];
          break;

      }
    } else {
      //Serial.println("No Command");
    }
  }
}

void readSerialAdd2Buffer() {
  //Read Testing Commands from USB Serial
  if (Serial.available() > 0) {
    //Serial.println("Reading Testing Command");
    String comString = "";
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      comString += inChar;
    }
    //Serial.println("TCommand: " + comString);
    if (isInputValid(comString)) {
      buildBuffer(comString);
      popCommands();

    } else {
      Serial.println("\nInvalid Testing Command");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ADCS Functions

/* Supported Commands ADCS Can Recieved
  //TODO
*/

void IMUInit() {
  //Initialize Default Scale of IMU/Mag
  //TODO init failure
  mag.init();
  imu.init();

  imu.enableDefault();
  mag.enableDefault();
}

void sendADCSCommand(String data) {
  //Send Command to ADCS Core
  //Serial.print("Command Sent to ADCS: <");
  //Serial.println(data + ">");
  char com[data.length() + 1];
  data.toCharArray(com, data.length() + 1);
  if (WireConnected) {
    Wire.beginTransmission(11); // transmit to device #11
    Wire.write(com);   // sends String
    Wire.endTransmission();    // stop transmitting
  }
}

void sectionReadToValue(String s, int * data, int dataSize) {
  //Convert Array of Strings <s> to Array of ints <data> with size <dataSize>
  for (int i = 0; i < dataSize; i++) {
    data[i] = (s.substring(0, s.indexOf(','))).toInt();
    s = s.substring(s.indexOf(',') + 1);
  }
}


bool requestFromADCS() {
  String res = "";
  bool success = false;
  if (WireConnected) {
    Wire.requestFrom(11, 40, true); // request 10 bytes from ADCS device #TODO
    //delay(50);
    int endTime = millis() + manualTimeout;
    //Serial.println("Here");

    //Read and Reformat
    //  ADCS_Active;
    if (Wire.available()) {
      success = true;
    }
    String res = "";
    while (Wire.available()) {
      res += (char)Wire.read();
    }
    res = res.substring(0, res.indexOf('|'));

    int data[12];
    sectionReadToValue(res, data, 12);
    //TODO ADCS Data Split
    //    MSH.MResets = data[0];
    //    MSH.AnalogTemp = data[1];
    //    MSH.LightSense = data[2];
    //    MSH.CurXDir = data[3];
    //    MSH.CurYDir = data[4];
    //    MSH.CurZDir = data[5];
    //    MSH.CurXPWM = data[6];
    //    MSH.CurYPWM = data[7];
    //    MSH.CurZPWM = data[8];
  }
  return success;
}


String buildIMUDataCommand() {
  // ex. gyro data: "11,3.653!12,2.553!13,-10!"
  String res = "";
  //Sends Info x1000
  res += "401," + String((long int)(1000 * MSH.Accel[0])) + "!";
  res += "402," + String((long int)(1000 * MSH.Accel[1])) + "!";
  res += "403," + String((long int)(1000 * MSH.Accel[2])) + "!";
  res += "404," + String((long int)(1000 * MSH.Gyro[0])) + "!";
  res += "405," + String((long int)(1000 * MSH.Gyro[1])) + "!";
  res += "406," + String((long int)(1000 * MSH.Gyro[2])) + "!";
  res += "407," + String((long int)(1000 * MSH.Mag[0])) + "!";
  res += "408," + String((long int)(1000 * MSH.Mag[1])) + "!";
  res += "409," + String((long int)(1000 * MSH.Mag[2])) + "!";
  return res;
}

void sendIMUToADCS() {
  String SCommand = buildIMUDataCommand();
  char SComCharA[SCommand.length() + 1];
  SCommand.toCharArray(SComCharA, SCommand.length() + 1);
  sendADCSCommand(SComCharA);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Radio Uplink/Downlink Functions




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Piksi GPS Functions



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// GomSpace Battery Functions

//p31u-6
//public typedef struct {
//  uint16_t pv[3]; //Photo-voltaic input voltage [mV]
//  uint16_t pc; //Total photo current [mA]
//  uint16_t bv; //Battery voltage [mV]
//  uint16_t sc; //Total system current [mA]
//  int16_t temp[4]; //Temp. of boost converters (1,2,3) and onboard battery [degC]
//  int16_t batt_temp[2]; //External board battery temperatures [degC];
//  uint16_t latchup[6]; //Number of latch-ups on each output 5V and +3V3 channel
//  //Order[5V1 5V2 5V3 3.3V1 3.3V2 3.3V3]
//  //Transmit as 5V1 first and 3.3V3 last
//  uint8_t reset; //Cause of last EPS reset
//  uint16_t bootcount; //Number of EPS reboots
//  uint16_t sw_errors; //Number of errors in the eps software
//  uint8_t ppt_mode; //0 = Hardware, 1 = MPPT, 2 = Fixed SW PPT.
//  uint8_t channel_status; //Mask of output channel status, 1=on, 0=off
//  //MSB - [QH QS 3.3V3 3.3V2 3.3V1 5V3 5V2 5V1] - LSB
//  // QH = Quadbat heater, QS = Quadbat switch
//} hkparam_t;

//p31u-8 and p31u-9
public typedef struct eps_hk_vi_t __attribute__((packed)) { //20 bytes
  uint16_t vboost[3]; //! Voltage of boost converters [mV] [PV1, PV2, PV3]
  uint16_t vbatt; //! Voltage of battery [mV]
  uint16_t curin[3]; //! Current in [mA]
  uint16_t cursun; //! Current from boost converters [mA]
  uint16_t cursys; //! Current out of battery [mA]
  uint16_t reserved1; //! Reserved for future use
};

//typedef struct __attribute__((packed)) {
//  uint16_t curout[6]; //! Current out (switchable outputs) [mA]
//  uint8_t output[8]; //! Status of outputs**
//  uint16_t output_on_delta[8]; //! Time till power on** [s]
//  uint16_t output_off_delta[8]; //! Time till power off** [s]
//  uint16_t latchup[6]; //! Number of latch-ups
//} eps_hk_out_t;
////**[6] is BP4 heater, [7] is BP4 switch
//
//typedef struct __attribute__((packed)) {
//  uint32_t wdt_i2c_time_left; //! Time left on I2C wdt [s]
//  uint32_t wdt_gnd_time_left; //! Time left on I2C wdt [s]
//  uint8_t wdt_csp_pings_left[2]; //! Pings left on CSP wdt
//  uint32_t counter_wdt_i2c; //! Number of WDT I2C reboots
//  uint32_t counter_wdt_gnd; //! Number of WDT GND reboots
//  uint32_t counter_wdt_csp[2]; //! Number of WDT CSP reboots
//} eps_hk_wdt_t;
//
//typedef struct __attribute__((packed)) {
//  uint32_t counter_boot; //! Number of EPS reboots
//  int16_t temp[6]; //! Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BATT0, BATT1]
//  uint8_t bootcause; //! Cause of last EPS reset
//  uint8_t battmode; //! Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4 = full]
//  uint8_t pptmode; //! Mode of PPT tracker [1=MPPT, 2=FIXED]
//  uint16_t reserved2;
//} eps_hk_basic_t;

void ByteArrayToStructure(byte[] b)
{
  eps_hk_vi_t tmp; //Re-make the struct
  memcpy(&tmp, b, sizeof(tmp));
  //ONLY WORKS WITH THE SAME ENDIANESS
  //Serial.println(tmp.
}

void fetchHouseKeeping() {
  //Fetch eps_hk_vi_t
  Wire.beginTransmission(0x0C);
  Wire.write(8);
  Wire.write(1);
  delayMicroseconds(5);

  byte temp[100] = {0};
  byte hk_vi_t[20] = {0};

  //Read Retu
  int i = 0
  while (Wire.available()) {
    temp[i] = Wire.read();
    i++;
  }
  i = 0;





}


void rebootGS() {
  //Sends Magic Sequence to Board to reboot the GS
  int outgoingByte[] = {0x80};
  int outgoingByte1[] = {0x07};
  int outgoingByte2[] = {0x80};
  int outgoingByte3[] = {0x07};
  Wire.write(outgoingByte[0]);
  Wire.write(outgoingByte1[0]);
  Wire.write(outgoingByte2[0]);
  Wire.write(outgoingByte3[0]);

  rtrn = Wire.endTransmission(false);
}


void setGomSpaceSlaveMode() {

}

bool pingGS() {
  //Pings the GomSpace Board to verify that its functioning, ping returns whatever value is sent to it.
  Wire.beginTransmission(0x0C);
  Wire.write(1);
  Wire.write(7);
  delayMicroseconds(5);

  uint8_t r;
  while (Wire.available()) {
    r = Wire.read();
  }
  if (r == 7) {
    return true;
  } else {
    return false;
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Stall() {
  stall = true;
  long start = millis();
  Serial.println("Stall Delay");
  while (millis() - start < 3000) { //(stall) {
    delay(80);
    Serial.print(".");
  }
}

//// Main Loop

void setup() {

  //Start Connections and Create MSH
  Serial.begin(9600);
  Wire.begin(); //Start i2c as master
  MSH = masterStatus();
  cBuf = commandBuffer();

  //Set Pinmode Registers
  initalizePinOut();

  //Start IMU/Mag/Gyro
  Serial.println(F("\nStarting IMU"));
  IMUInit();

  //Start Piksi GPS

  //Start Quake Radio


  int endT = millis() + manualTimeout;
  MSH.State = NORMAL_OPS;
  MSH.NextState = NORMAL_OPS;
}

void loop() {
  readSerialAdd2Buffer(); //Testing Command Input

  //Mode Controller
  //  Determined MSH.

  switch (MSH.State) {
    case (NORMAL_OPS): {

        //Collect Sensor Data
        SensorDataCollect();

        //GNC Calculation

        //Thruster Firing

        //ADCS Calculation


        //Downlinks
        if (millis() - lastDLTime >= DLTime || commandedDL) {
          if (testRDL || commandedDL) {

            //TODO DownLinks

          }
          lastDLTime = millis();
          if (commandedDL) {
            commandedDL = false;
          }
        }

        //Test ADCS Communication
        if (true) { //MSH.hardwareAvTable[10]) {
          unsigned long t = millis();
          if (millis() - lastADCSComAttempt >= ADCSComTime) {
            lastADCSComAttempt = millis();
            Serial.print("<IMU>");
            sendIMUToADCS();
            bool ADCSResponse = requestFromADCS();
            if (ADCSResponse) {
              lastADCSComTime = millis(); //Reset Timeout if Com is successful
            } else {
              Serial.print(F("No Reply From ADCS for "));
              Serial.print((millis() - lastADCSComTime) / 1000.0);
              Serial.println(F(" seconds"));
              //            }
            }
          }
        }

        //ADCS Testing Display
        if (millis() - LastSpinCheckT > SpinCheckTime) {
          Serial.print(F("<G:") + String(MSH.Gyro[0], 2) + "|" +
                       String(MSH.Gyro[1], 2) + "|" +
                       String(MSH.Gyro[2], 2) + ">");
          Serial.print(F("<MG:") + String(MSH.Mag[0], 2) + "|" +
                       String(MSH.Mag[1], 2) + "|" +
                       String(MSH.Mag[2], 2) + ">");
          Serial.print(F("<AC:") + String(MSH.Accel[0], 2) + "|" +
                       String(MSH.Accel[1], 2) + "|" +
                       String(MSH.Accel[2], 2) + ">");
          LastSpinCheckT = millis();
        }


        //Low Power Detection
        if (millis() - LastBattCheck > BattCheckTime) {

          //TODO GomSpace Packet Fetch

          Serial.print("<B:" + String(MSH.Battery) + ">");
          if (MSH.Battery <= LV_Threshold) {
            //MSH.NextState = LOW_POWER; //TODO
            lowPowerEntry = millis();
          }
          LastBattCheck = millis();
        }

        //Eclipse Detection
        //TODO
        if (false) {
          MSH.NextState = ECLIPSE;
        }

        //Thermal Protection/Control //TODO
        if (millis() - LastThermalCheck > ThermalCheck) {
          LastThermalCheck = millis();
          float T_avg = 0; //TODO Max? Min? Avg?
          for (int i = 0; i < 4; i++) {
            T_avg += MSH.Temp[i];
          }
          T_avg = T_avg / 4.0;
          Serial.print("<T" + String((float)T_avg) + ">");
        }
        break;
      }
    case (DORMANT_CRUISE):
      //30 min Dormant Cruise
      if (millis() > 45 * 60 * 1000) {
        MSH.NextState = INITALIZATION;
        initEntry = millis();
      } else {
        delay(10000);
      }
      break;

    case (INITALIZATION): {
        //TODO Checkout

        if (millis() - initEntry > (long)2700000) { //Force to Normal Ops
          //call downlink function
          //TODO
          MSH.NextState = NORMAL_OPS;
        }
        break;
      }

    case (ECLIPSE): {
        MSH.NextState = NORMAL_OPS;
        //TODO
      }


    case (SAFE_HOLD):
      MSH.NextState = NORMAL_OPS;
      //TODO
      break;

  }
  MSH.State = MSH.NextState;

  //Testing Iterators
  cycle++;

  if (true && ((millis() - LastTimeTime) > TimeTime)) { //Prevent Screen Spam
    long t = millis();
    Serial.print("[" + String((millis() - LastTimeTime) / 1000.0) + "]"); //Cycle Lag
    String s = ("\n[System Time: " + String(t / (long)(60 * 60 * 1000)) + ":" +
                String(t / ((long)60 * 1000) % ((long)60 * 1000)) + ":");
    if (((t / 1000) % ((long)1000) % 60) < 10) {
      s += '0';
    }
    s += (String((t / 1000) % ((long)1000) % 60) +
          "][" + String(MSH.State) + "]");
    s += ("[" + String(freeRam() / 1024.0, 3) + "kB]");
    Serial.print(s);
    LastTimeTime = t;
    cycle = 1;
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////












