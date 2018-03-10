
#include "Quake_D.h"

// When defined, debugging information is output of Serial
#define DEBUG

// ------------------------------------
// qlocate class implementation
// ----------------

// ----------------
// public members

qlocate::qlocate(HardwareSerial *port, int timeout) {
  // Initialize class variables
  this->sbdix_running = false;
  this->port = port;
  // Configure QLocate port settings
  this->port->begin(19200);
  this->port->setTimeout(timeout);
}

int qlocate::config() {
  // Ensure no ongoing sbdix session
  if(sbdix_running) return -2;
  // Function success flag
  int success = 0;
  // Restore factory defaults
  port->print(F("AT&F0\r"));
  int success = consume(F("\r\nOK\r\n"));
#ifdef DEBUG
  Serial.println("qlocate > config > AT&F0 status=" + String(success));
#endif
  // Disable flow control, disable DTR, disabl echo, set numeric rasponses, and
  // disable "RING" alerts
  port->print(F("AT&K0;&D0;E0;V0;+SBDMTA=0\r"));
  success |= consume(F("AT&K0;&D0;E0;V0;+SBDMTA=0\r0\r"));
#ifdef DEBUG
  Serial.println("                 > AT&K0;&D0;E0;V0;+SBDMTA=0 status="
        + String(success));
#endif
  // Clear QLocate MO and MT buffers
  port->print(F("AT+SBDD2\r"));
  success |= consume(F("0\r\n0\r"));
#ifdef DEBUG
  Serial.println("                 > AT+SBDD2 status="
        + String(success));
#endif
  return success;
}

int qlocate::write_mo_buf(char const *c, int len) {
  // Ensure no ongoing sbdix session
  if(sbdix_running) return -2;
  // Request to write binary data
  port->print(F("AT+SBDWB="));
  port->print(String(len));
  port->write('\r');
  // Listen for QLocate's ready response
  int code = this->consume(F("READY\r\n"));
  if(code != 1) return code;
  // Write binary data to QLocate
  port->write(c, len);
  // Write checksum to QLocate
  short s = checksum(c, len);
  port->write((char)(s >> 8));
  port->write((char) s);
#ifdef DEBUG
  Serial.print("qlocate > sbdwb > mes=");
  for(int i = 0; i < len; i++)
    Serial.print(c[i], HEX);
  Serial.print((char)(s >> 8), HEX);
  Serial.println((char) s, HEX);
#endif
  // Process QLocate response
  char buf[4];
  len = port->readBytes(buf, 3);
  buf[len] = '\0';
#ifdef DEBUG
  Serial.print("                > res=");
  for(int i = 0; i < len; i++)
    Serial.print(buf[i], HEX);
  Serial.println("\n                > return=" + String(*buf));
#endif
  if(buf[1] != '\r' || buf[2] != '\n') return -1;
  return buf[0] - '0';
}

// ----------------
// protected members

int qlocate::consume(String res) {
  // Read in current port input
  char buf[res.length()];
  int len = port->readBytes(buf, res.length());
  buf[len] = '\0';
// #ifdef USBCONNECT
//  Serial.print("qlocate > consume > exp=");
//  for(unsigned int i = 0; i < res.length(); i++)
//    Serial.print(res.charAt(i), HEX);
//  Serial.print("\nqlocate > consume > res=");
//  for(int i = 0; i < len; i++)
//    Serial.print(buf[i], HEX);
//  Serial.println();
// #endif
// Determine status code
  if(len == 0) return -1;
  return (!res.equals(String(buf)) || port->available());
}

short qlocate::checksum(char const *c, int len) {
  short s = 0;
  char const *const cf = c + len;
  while(c < cf) s += *(c++);
  return s;
}

// ----------------
// End qlocate class
// ------------------------------------
