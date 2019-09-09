// Speed Control Sample for Kangaroo
// Copyright (c) 2013 Dimension Engineering LLC
// See license.txt for license details.
// Edit: Valerie Pober -- 7/17/2019 Teensy, RC/ROS switch test

//RC/ROS switch via RC_SIGNAL const
//test of ramp limit, second param for s function for max rate of change of vel
#include <Kangaroo.h>

// Teensy TX (pin 1) goes to Kangaroo S1
// Teensy RX (pin 0) goes to Kangaroo S2
// Teensy GND         goes to Kangaroo 0V
// Teensy 5V          goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Arduino)

#define KSERIAL Serial1
#define switchRC 10
#define out 11
#define RC_SIGNAL LOW

KangarooSerial  K(KSERIAL);
KangarooChannel K1(K, '1');
KangarooChannel K2(K, '2');

int RC = HIGH;

void setup()
{
  Serial.begin(9600);
  KSERIAL.begin(9600);

  pinMode(switchRC, INPUT);
  pinMode(out, OUTPUT);

  digitalWrite(out, RC_SIGNAL);

  RC = digitalRead(switchRC);
 
  if(RC == HIGH)
  {
    K1.start();
    K1.home().wait();
  }
}

// .wait() waits until the command is 'finished'. For speed, this means it reached near the
// requested speed. You can also call K1.s(speed); without .wait() if you want to command it
// but not wait to get up to speed. If you do this, you may want to use K1.getS().value()
// to check progress.
void loop()
{

  if(RC == HIGH)
  {
    RC_control();
  }else{
    ROS_control();
  }
}

void RC_control()
{
  long minimum = K1.getMin().value();
  long maximum = K1.getMax().value();
  long speedVal = 100;
  
  K1.s(speedVal,1000).wait(); //second param = ramp limit
  delay(5000);
  
  K1.s(-speedVal,1000).wait();
  delay(5000);
}

void ROS_control()
{
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
