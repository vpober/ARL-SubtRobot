// Mixed Mode Sample for Kangaroo
// Modified from Kangaroo Arduino library example MixedMode.ino

#include <Kangaroo.h>
#include <string>
#include <unistd.h>


using namespace std;

string port = "/dev/ttyUSB0";
unsigned long baud = 9600;
serial::Serial serial_port(port, baud, serial::Timeout::simpleTimeout(1000));
SerialStream stream(serial_port);
KangarooSerial  K(stream);

// Mixed mode channels on Kangaroo are, by default, 'D' and 'T'.
KangarooChannel Drive(K, 'D');
KangarooChannel Turn(K, 'T');

void delay(int milliseconds)
{
    useconds_t microseconds = milliseconds * 1000;
    usleep(microseconds);
}

void setup()
{
  Drive.start();
  Turn.start();
  
  Drive.si(0);
  Turn.si(0);
}

void loop()
{
  Drive.s(300);
  delay(1000);
  Drive.s(0); 
  delay(1000);
  
  Turn.s(300);  
  delay(1000);
  Turn.s(0);
  delay(1000);
}

int main() 
{
    setup();    
    while (true) {
        loop();
    }    
    return 0;
}
