//
//  QLocate.hpp
//  QLocate
//
//  Created by Kyle Krol (kpk63@cornell.edu) on 3/04/18.
//  Pathfinder for Autonomous Navigation
//  Cornell University
//

#include "QLocate.hpp"

// Remove to prevent debug output
#define DEBUG

/*! QLocate::Message implementation */ // ----------------------------------------

QLocate::Message::Message() {
  this->length = 0;
}

QLocate::Message::Message(QLocate::Message const &mes) {
  this->length = mes.length;
  for(int i = 0; i < this->length; i++)
    this->mes[i] = mes.mes[i];
}

int QLocate::Message::get_length() const {
  return this->length;
}

char &QLocate::Message::operator[](int i) {
  return this->mes[i];
}

void QLocate::Message::copy_message(char *c) const {
  for(int i = 0; i < this->length; i++)
    c[i] = this->mes[i];
}

/*! QLocate implementation */ // -----------------------------------------------

QLocate::QLocate(HardwareSerial *const port, int timeout) : port(port) {
  // Initialize class variables
  this->sbdix_running = false;
  this->message = Message();
  // Configure QLocate port settings
  this->port->begin(19200);
  this->port->setTimeout(timeout);
}

bool QLocate::sbdix_is_running() {
  return this->sbdix_running;
}

int const *QLocate::get_sbdix_response() {
  return this->sbdix_r;
}

QLocate::Message &QLocate::get_message() {
  return this->message;
}

int QLocate::config() {
  // Ensure no ongoing sbdix session
  if(sbdix_running) return -1;
  // Clear serial buffer
  port->clear();
  // Restore factory defaults
  port->print(F("AT&F0\r"));
  int success = consume(F("\r\nOK\r\n"));
  if(success != 0) {
    port->clear();
    port->print(F("AT&F0\r"));
    success = consume(F("\r\nOK\r\n"));
  }
  if(success == -1) return success;
#ifdef DEBUG
  Serial.println("config > AT&F0 status=" + String(success));
#endif
  // Disable flow control, disable DTR, disabl echo, set numeric rasponses, and
  // disable "RING" alerts
  port->print(F("AT&K0;&D0;E0;V0;+SBDMTA=0\r"));
  success |= consume(F("AT&K0;&D0;E0;V0;+SBDMTA=0\r0\r"));
#ifdef DEBUG
  Serial.println("       > AT&K0;&D0;E0;V0;+SBDMTA=0 status="
      + String(success));
#endif
  // Clear QLocate MO and MT buffers
  port->print(F("AT+SBDD2\r"));
  success |= consume(F("0\r\n0\r"));
#ifdef DEBUG
  Serial.println("       > AT+SBDD2 status=" + String(success));
#endif
  return success;
}

int QLocate::sbdwb(char const *c, int len) {
  // Ensure no ongoing sbdix session
  if(sbdix_running) return -1;
  // Clear serial buffer
  port->clear();
  // Request to write binary data
  port->print(F("AT+SBDWB="));
  port->print(String(len));
  port->write('\r');
  int code = consume(F("READY\r\n"));
#ifdef DEBUG
  Serial.println("load_mo > write_req_res= " + String(code));
#endif
  if(code != 0) return code;
  // Write binary data to QLocate
  port->write(c, len);
  short s = checksum(c, len);
  port->write((char)(s >> 8));
  port->write((char) s);
  port->flush();
#ifdef DEBUG
  Serial.print("        > mes=");
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
  Serial.print("        > res=");
  for(int i = 0; i < len; i++)
    Serial.print(buf[i], HEX);
  Serial.println("\n        > return=" + String(*buf));
#endif
  if(buf[1] != '\r' || buf[2] != '\n') return -1;
  return buf[0] - '0';
}

int QLocate::run_sbdix() {
  // Ensure no ongoing sbdix session
  if(sbdix_running) return -1;
  // Request sbdix session
  port->print(F("AT+SBDIX\r"));
  sbdix_running = true;
#ifdef DEBUG
  Serial.println("run_sbdix > return= 0");
#endif
  return 0;
}

// int parsing helper function
int parse_int(char **const c, char const term, int *const i) {
  *i = 0;
  int count = 0;
  while(**c != term) {
    *i = 10 * *i + (*((*c)++) - '0');
    if(count++ > 6) {
    #ifdef DEBUG
      Serial.println("ERROR: parse_int infinite loop");
    #endif
      return -1; // prevent infinite loop
    }
  }
  return 0;
}

int QLocate::end_sbdix() {
  // Ensure sbdix is sbdix is running and data is available
  if(!sbdix_running || !port->available()) return -1;
  // Parse quake output
  char buf[75];
  char *ptr = &buf[7]; // skip "+SBDIX:"
  port->readBytesUntil('\r', buf, 74);
  if(parse_int(&ptr, ',',  sbdix_r    ) == -1) return 1;
  ptr++;
  if(parse_int(&ptr, ',',  sbdix_r + 1) == -1) return 1;
  ptr++;
  if(parse_int(&ptr, ',',  sbdix_r + 2) == -1) return 1;
  ptr++;
  if(parse_int(&ptr, ',',  sbdix_r + 3) == -1) return 1;
  ptr++;
  if(parse_int(&ptr, ',',  sbdix_r + 4) == -1) return 1;
  ptr++;
  if(parse_int(&ptr, '\r', sbdix_r + 5) == -1) return 1;
  return 0;
}

int QLocate::sbdrb() {
  // Ensure no ongoing sbdix session and flush buffer
  if(sbdix_running) return -1;
  port->clear();
  // Request data
  Serial3.print(F("AT+SBDRB\r"));
  // Capture incoming data and check checksum
  short s;
  if(2 != Serial3.readBytes((char *) &s, 2))
    return 1; // Message length read fails
  unsigned short size = (s & 0xFF) << 8 | (s >> 8);
#ifdef DEBUG
  Serial.println("sbdrb > recieving message size= " + String(size));
#endif
  if(size + 2 != (unsigned short) Serial3.readBytes(message.mes, size + 2))
    return 0; // Message read fails
  s = checksum(message.mes, size);
  if(((s & 0xFF) << 8 | (s >> 8)) != *(short *)(message.mes + size)) {
    return 1; // Checksum error detected
  #ifdef DEBUG
    Serial.println("");
  #endif
  }
  // Format as a string // TODO : Excecute included AT commands here
  message.mes[size] = '\0';
  return 0;
}

int QLocate::consume(String res) {
  // Read in current port input
  char buf[res.length() + 1];
  int len = port->readBytes(buf, res.length());
  buf[len] = '\0';
  // Determine status code
  if(len == 0) return -1;
  return (!res.equals(String(buf)) || port->available());
}

short QLocate::checksum(char const *c, int len) {
  short s = 0;
  char const *const cf = c + len;
  while(c < cf) s += *(c++);
  return s;
}
