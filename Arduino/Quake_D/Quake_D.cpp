
#include "Quake_D.h"
#include <Wire.h>

// ------------------------------------
// helper functions
// ----------------

bool str_equals(const char *r, const char *s, const char t) {
  while(*r == *s && *r != t) {
    r++;
    s++;
  }
  return (*r == t && *s == t);
}

bool str_equals(const char *r, const char *s) {
  return str_equals(r, s, '\0');
}

// ------------------------------------
// quake class implementation
// ----------------

quake::quake(long timeout) {
  Serial3.begin(9600);
  Serial3.setTimeout(timeout);
  // TODO : Potentially control Vcc here too
}

int quake::configure() {
  // TODO
  return 0;
}
