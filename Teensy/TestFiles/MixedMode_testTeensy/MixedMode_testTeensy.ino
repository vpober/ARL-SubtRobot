// Mixed Mode Sample for Kangaroo
// Copyright (c) 2013 Dimension Engineering LLC
// See license.txt for license details.
//Edit: Valerie Pober -- 8/26/2019 Teensy

#include <Kangaroo.h>
//#include<Timer.h>

//Kangaroo Serial channel- Teensy pins 0, 1
#define KSERIAL Serial1

// Mixed mode channels on Kangaroo are, by default, 'D' and 'T'.
KangarooSerial  K(KSERIAL);
KangarooChannel Drive(K, 'D');
KangarooChannel Turn(K, 'T');

void setup()
{
  Serial.begin(9600);
  KSERIAL.begin(9600);

  Drive.start();
  Turn.start();

  Drive.si(0);
  Turn.si(0);
}

void loop()
{
//    Drive.s(50);
//  delay(1000);
//  Turn.s(50);
//  delay(1000);

  // Drive 500 ticks (relative to where we are right now), then wait 1 second.
  Drive.pi(500, 100).wait();


//  KangarooStatus d = Drive.getS();
//  KangarooStatus t = Turn.getS();
//  Serial.println("ERROR Status d, t");
//  Serial.println(d.error());
//  Serial.println(t.error());

  Drive.s(0);
  // Turn 500 ticks (relative to where we are right now), then wait 1 second.
  Turn.pi(500, 100).wait();
  //Turn.s(50).wait();
  //delay(1000);
  Turn.s(0);
}
