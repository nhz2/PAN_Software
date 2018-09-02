
#include "QLocate.hpp"

QLocate q(&Serial3, 2000);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  char c[] = {'R','u','n','n','i','n','g',' ','t',',',
      'e','s','t','s',' ','a','g','a','i','n','!',' ',
      'H','e','l','l','o',' ','f','r','o','m',' ','Q',
      'L','o','c','a','t','e',' ','o','n',' ','t','h',
      'e',' ','E','n','g','i','n','e','e','r','i','n',
      'g',' ','Q','u','a','d',':',')'};
  q.config();
  q.sbdwb(c, sizeof(c));
  q.run_sbdix();
  
  int res = 0;
  do {
    delay(500);
    res = q.end_sbdix();
  } while(res == -1);
  Serial.println("end_sbdix res= " + String(res));
  for(int i = 0; i < 6; i++)
    Serial.print(String(q.get_sbdix_response()[i]) + " ");
  Serial.println();
}

void loop() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
