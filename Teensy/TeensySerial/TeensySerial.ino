#define KSERIAL Serial1
void setup() {

  Serial.begin(9600);
  KSERIAL.begin(9600);
}

void loop() {

}

void serialEvent(){

  while(Serial.available()){
    int dataByte;
    dataByte = Serial.read();
    KSERIAL.write(dataByte);
  }
}

void serialEvent1(){

  while(KSERIAL.available() > 0 ){
    int dataByte;
    dataByte = KSERIAL.read();
    Serial.write(dataByte);
  }
}
