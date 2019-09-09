//original implementation without interrupts
#include <Kangaroo.h>
#define KSERIAL Serial1
void setup() {

  Serial.begin(9600);
  KSERIAL.begin(9600);
}

void loop() {

  int dataByte;

  if(Serial.available()>0){
    dataByte = Serial.read();
    KSERIAL.write(dataByte);
  }

  if(KSERIAL.available() > 0){
    dataByte = KSERIAL.read();
    Serial.write(dataByte);
  }
}
