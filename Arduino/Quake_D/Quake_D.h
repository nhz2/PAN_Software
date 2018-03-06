
#ifndef Quake_D_hpp
#define Quake_D_hpp

#include <Wire.h>

// TODO : Define error codes for end_sbdix()

// TODO : Comment on general error code layout (<0 library defined >0 Quake)

class quake {

public:

  /*! Initializes Serial3 ports on the Teensy. Sets the Serial3 timeout value
   *  used with readBytesUntil(...) and readBytes(...). Does not initiate any
   *  communication with the Quake itself. See configure().
   *    @param timeout timeout value for Serial3
   */
  quake(long timeout); // TODO : May toggle Quake Vcc to high as well

  // void restart_quake(); // TODO : Implement if Vcc control pin avaliable

  /*! Pings the Quake with "AT", listens for the "\r\nOK.\r\n" response, and
   *  sets the Quake's responses to the numeric format. This method must be
   *  called before using any other functionality in the quake class every time
   *  the Quake restarts.
   *  Returns 1 if succesful and 0 otherwise.
   *  Failure of this method implies the Teensy isn't communicating properly
   *  with the Quake radio at all.
   */
  int configure(); // TODO : Look into disabling ring alerts

  /*! Signals whether a new message from sbdix is avaliable. Once one is
   *  avaliable, this method will return true until a function of the form
   *  read_xxx() has been called. Once such a function is called this method
   *  will return true once more when another message has been recieved.
   */
  bool has_new_mes() {
    return this->new_data;
  }

  /*! Returns mesi[] - the string formatted version of the latest message
   *  recieved by the Quake. The string contains the { message } portion of
   *  the the raw message. The data is terminated by 0x00.
   *    @return string message data
   */
  const unsigned char *get_mes() {
    this->new_data = false;
    return this->mesi;
  }

  /*! Writes a message to the Quake outgoing data buffer. An sbdix session must
   *  be run for the data to be sent. If previous data has not yet been sent
   *  via sbdix, the Quake's outgoing buffer will be written over. Quake
   *  response code is returned.
   *    @param c array pointer to outgoing message
   *    @param size number of bytes in outgoing message
   *    @return Quake response code
   *  // TODO : Explain -1 Quake timeout issue
   */
  int write_message(unsigned char const *c, unsigned short size);

  /*! Begins an sbdix session on the Quake. The following task queued:
   *    1. The Quake attempts registration with Iridium.
   *    2. A loaded message will be transmitted to Iridium.
   *    3. Any pending messages will be downloaded from the Iridium.
   *    4. Sets in_sbdix to true.
   *  The Quakes response will take ~ // TODO : Completion time estimate.
   *  It is recommended to wait some time before querying the result of the
   *  sbdix session (see end_sbdix()).
   */
  void start_sbdix() {
    Serial3.print("AT+SBDIX\r");
    this->in_sbdix = true;
  }

  /*! Attempts to read the result of an sbdix session from the Quake. The
   *  following tasks are executed:
   *    1. Read the sbdix response from the Serial3 buffer.
   *    2. Download a message from the Quake buffer if present and set new_data
   *       to true if neccesary.
   *    3. Format error response if any. // TODO : Format error codes
   *    4. Sets in_sbdix to false.
   *    @return sbdix error response
   */
  int end_sbdix();

private:

  /*! Indicates if an sbdix session is ongoing */
  bool in_sbdix;

  /*! Indicates if a new message has been recieved via sbdix */
  bool new_data;

  /*! Input buffer for Serial3 reads from quake */
  unsigned char mesi[62]; // TODO : Size depends on Serial3 buffer
                          // Serial3 buffer 64 bytes?

  /*! Executes the sbdrb Iridium command to read in an incoming message. The
   *  message is then formatted and stored in the mesi[] variable. It can be
   *  retrieved with get_mes().
   *    @return success flag
   */
  bool sbdrb();

};

#endif
