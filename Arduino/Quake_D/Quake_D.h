
#ifndef Quake_D_hpp
#define Quake_D_hpp

#include <Wire.h>

/*! // TODO : Class specification
 */
class qlocate {

public:

  /*! The quake constructor is responsible for configuring the serial port
   *  dedicated to communicating with the QLocate and initializing class
   *  variables to their defaults - sbdix_running is set to false.
   */
  qlocate(HardwareSerial *port, int timeout);

  /*! This manipulates the settings of the QLocate and sets it to communicate
   *  via the 3 pin interface. It needs to be called prior to calling any other
   *  function which interfaces with the QLocate. This method sends the QLocate
   *  the following AT commands:
   *    1. AT&F0       - restores to factory defaults
   *    2. AT&K0       - disable RTS & CTS flow control
   *    3. AT&D0       - ignores the DTR pin
   *    4. ATE0        - disables echo
   *    5. ATV0        - sets responses to numeric mode
   *    6. AT+SBDMTA=0 - disables RING alerts
   *    7. +SBDD2      - clears the QLocate's buffer
   *  The function returns -1 if no response is heard from the QLocate, 1 if the
   *  response received was not expected, and 0 if the function was succesful.
   *    @return status code of -1, 0, or 1
   */
  int config();

  /*! Writes data to the QLocate's MO buffer. The data held here will be
   *  transmitted to Iridium during the next sbdix session. The MO buffer can
   *  only hold a single message at a time with a max length of 340 bytes.
   *    @param c pointer to binary data
   *    @param len length of binary data
   *    @return status code
   */
  int write_mo_buf(char const *c, int len);

  /*! Initiates an sbdix session. Data that you wish to transmit should be
   *  loaded into the QLocate with sbdwb(...) prior to initiating an sbdix
   *  session. The sbdix session can be ended with end_sbdix(). Note that while
   *  the sbdix session is in session, no other commands may be sent to the
   *  QLocate.
   *    @return status code
   */
  int run_sbdix() {
    // Ensure no ongoing sbdix session
    if(sbdix_running) return -2;
    // Clear port buffer and initiate sbdix session
    port->clear();
    port->print(F("AT+SBDIX\r"));
  }

  int end_sbdix();

protected:

  /*! Flag for currently running sbdix session */
  bool sbdix_running;

  /*! Serial port dedicated to communication with the QLocate */
  HardwareSerial *port;

  /*! This function consumes an expected response from the serial port
   *  associated with the QLocate. The method returns 0 if the expected response
   *  is read from the port, 1 if the response is unexpected, and -1 if no
   *  response is read before the call to port->readBytes() times out.
   *    @param res the expected string response from the quake
   *    @return status code of -1, 1, or 1
   */
  int consume(String res);

  /*! Calculates the checksum of binary data according to Iridium requirements.
   *  Be aware that the returned short is in little endian and the checksum is
   *  is written to the quake in big endian - on the teensy 3.5 at least.
   */
  short checksum(char const *c, int len);

};

#endif
