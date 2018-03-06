
#include "Quake_D.h"

// ------------------------------------
// Helper functions
// -----------------

/*! Calculates a sbdix message checksum according to the Iridium specifications.
 *  This sums all bytes in the message into a 16 bit value and returns the sum
 *  as the checksum.
 *    @param c start of data array
 *    @param size size of the data array
 *    @return calculated checksum value
 */
unsigned short calc_checksum(unsigned char const *c, int size) {
  unsigned const char *cf = c + size;
  unsigned short checksum = 0;
  while(c < cf)
    checksum += *(c++);
  return checksum;
}

/*! Converts a comma separated series of numbers into an int aray. The passed
 *  string must terminate with '\0' and not contain any charactars except [0-9]
 *  and ','. Two comma cannot come one after another (i.e ",,").
 *    @param c comma separated sequence string
 *    @param i destination int array
 */
void str_to_ints(unsigned char *c, int *i) { // TODO : Test this method
  while(*c != '\0') {
    int n = 0;
    while(*c >= '0' && *c <= '9')
      n = (n << 3) + (n << 1) + (*(c++) - '0'); // 10 * n + c
    *(i++) = n;
    c++;
  }
}

// ----------------
// End helper functions
// ------------------------------------
// quake class implemenation
// ----------------

// TODO : As a general note, consider flushing Serial3 before some functions

// TODO : Add in_sbdix state checks to most methods to avoid corruption

quake::quake(long timeout) {
  this->in_sbdix = 0;
  this->new_data = 0;
  Serial3.begin(9600);
  Serial3.setTimeout(timeout);
  // TODO : Potentially control Vcc here too
}

int quake::configure() { // TODO : Disable ring alerts
  // Ensure Quake is active
  Serial3.print(F("AT\r"));
  unsigned char res[8]; res[7] = '\0';
  int len = Serial3.readBytes(res, 7);
  if(len != 7 || !String((char *)res).equals(F("\r\nOK.\r\n")))
    return 0;
  // Set responses to numeric
  Serial3.print(F("ATV0\r")); // TODO : Ensure no response
  // Success
  return 1;
}

int quake::write_message(unsigned char const *c, unsigned short size) {
  // Alert Quake you would like to perform SBDWB
  Serial3.print(F("AT+SBDWB="));
  Serial3.write((char *)&size, 2); // TODO : Check memory layout
  Serial3.write('\r');
  // Wait for the ready response from Quake
  unsigned char res[8]; res[7] = '\0';
  int len = Serial3.readBytes(res, 7); res[7] = '\0';
  if(len != 7 || !String((char *)res).equals(F("READY\r\n")))
    return -1;
  // Write message to Quake outgoing buffer
  Serial3.write(c, size);
  unsigned char checksum = calc_checksum(c, size);
  Serial3.write((char *)&checksum, 2); // TODO : Check memory layout
  // Wait for Quake response
  len = Serial3.readBytes(res, 2);
  if(len != 2)
    return -1;
  *res -= '0';
  // Handle fringe case of "1OK" resoonse
  if(*res == 1)
    Serial3.readBytes(res, 2); // TODO : Look into actual response
  // Return response code
  return *res;
}

int quake::end_sbdix() {
  // Check to see if Quake response has been recieved
  if(Serial3.available())
    return -1;
  // Load Quake response to memory
  unsigned char res[64]; // TODO : Adjust to Serial3 buffer Size
  int len = Serial3.readBytes(res, 64); // TODO : Serial3 buffer size
  if(len < 19) // TODO : Add error code to description
    return -2;
  // Parse Quake response codes
  int res_code[6];
  str_to_ints(res + 7, res_code); // + 7 skips "SBDIX:" prefix
  // Check for incoming message and read into memory if one exists
  if(res_code[2] == 1) {
    // Message available and execute sbdrb
    int attempts = 0;
    while(attempts++ < 5 && !this->sbdrb()); // TODO : Adjust attempt limit
  } else if(res_code[2] == 2) {
    // Message downlink failed
    // TODO : failed downlink
  }
  // TODO : Further error code handling and return code formatting
  return 0;
}

bool quake::sbdrb() {
  // Send command
  Serial3.print(F("AT+SBDRB\r"));
  // Capture incoming data and check checksum
  unsigned short size;
  if(2 != Serial3.readBytes((char *)&size, 2))
    return 0; // Message length read fails
  if(size + 2 != (unsigned short) Serial3.readBytes(this->mesi, size + 2))
    return 0; // Message read fails
  if(calc_checksum(this->mesi, size) != *(unsigned char *)(this->mesi + size))
    return 0; // Checksum error detected
  // Format as a string // TODO : Excecute included AT commands here
  this->mesi[size] = '\0';
  return 1;
}

// ----------------
// End quake class
// ------------------------------------
