
#include "QLocate.hpp"

QLocate q(&Serial3, 2000);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.println(q.config());
  char c[] = {'h','e','l','l','o'};
  Serial.println(q.stage_mes(c, 5));

  /*
  char res[100];

  Serial3.print("AT&F0\r");
  Serial3.flush();
  res[Serial3.readBytes(res, 99)] = '\0';
  Serial.println(String(res));

  Serial3.print("AT&K0;&D0;E0;V0;+SBDMTA=0\r");
  Serial3.flush();
  res[Serial3.readBytes(res, 99)] = '\0';
  Serial.println(String(res));*/
  
}

void loop() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
