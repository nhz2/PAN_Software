// ------------------------------------
// helper functions
// -----------------

// TODO : Comma separated char string to int array

// ------------------------------------
// quake class implemenation
// ----------------

#include "Quake_D.h"

// TODO : As a general note, consider flushing Serial3 before some functions

quake::quake(long timeout) {
  this->in_sbdix = 0;
  this->new_data = 0;
  Serial3.begin(9600);
  Serial3.setTimeout(timeout);
  // TODO : Potentially control Vcc here too
}

int quake::configure() {
  // Ensure Quake is active
  Serial3.print(F("AT\r"));
  unsigned char res[8]; res[7] = '\0';
  int len = Serial3.readBytesUntil('\0', res, 7);
  if(len != 7 || !String((char *)res).equals(F("\r\nOK.\r\n")))
    return 0;
  // Set responses to numeric
  Serial3.print(F("ATV0\r"));
  // Succes
  return 1;
}

int write_message(unsigned char const *c, int size) {
  // Alert Quake you would like to perform SBDWB
  Serial3.print(F("AT+SBDWB="));
  Serial3.print(size, DEC);
  Serial3.write('\r');
  // Wait for the ready response from Quake
  unsigned char res[8]; res[7] = '\0';
  int len = Serial3.readBytesUntil('\0', res, 7); res[7] = '\0';
  if(len != 7 || !String((char *)res).equals(F("READY\r\n")))
    return -1;
  // Write message to Quake outgoing buffer (calculate checksum too)
  unsigned const char *cf = c + size;
  unsigned short checksum = 0;
  while(c < cf)
    checksum += *(c++);
  Serial3.write(c, size);
  Serial3.write((char *)&checksum, 2);
  // Wait for Quake response
  len = Serial3.readBytesUntil('\r', res, 2);
  if(len != 2)
    return -1;
  *res -= '0';
  // Handle fringe case of "1OK"
  if(*res == 1)
    Serial3.readBytesUntil('\r', res, 3); // TODO : Look into actual response
  // Return response code
  return *res;
}

int end_sbdix() {
  // TODO
  return 0;
}
