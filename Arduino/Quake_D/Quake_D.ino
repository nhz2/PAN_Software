
#include "Quake_D.h"

qlocate q(&Serial3, 2000);

void setup() {
  Serial.begin(9600);
  q.config();
  char c[] = {'h','e','l','l','o'};
  int res = q.sbdwb(c, 5);
  Serial.println(res);
}

void loop() {
  delay(1000);
}
