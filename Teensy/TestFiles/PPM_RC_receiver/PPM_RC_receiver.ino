//read ppm signals with pulse position library
#include <PulsePosition.h>

#define T_pin 5
#define A_pin 6

PulsePositionInput T_Ch3; 
PulsePositionInput A_Ch1; 
void setup() {
  
  Serial.begin(9600);
  
  T_Ch3.begin(T_pin);
  A_Ch1.begin(A_pin);

}

void loop() {
  int i;
  
  for(i = 0; i < T_Ch3.available(); i++)
  {
    float data;
    data = T_Ch3.read(i);
    Serial.print("CH3, ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data);
  }

}
