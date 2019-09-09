// Speed Control Sample for Kangaroo
// Modified from Kangaroo Arduino library example SeedControl.ino

#include <Kangaroo.h>
#include <unistd.h>

using namespace std;

string port = "/dev/ttyACM0";
unsigned long baud = 38400;

serial::Serial serial_port(port, baud, serial::Timeout::simpleTimeout(1000));
SerialStream stream(serial_port);
KangarooSerial  K(stream);
KangarooChannel K1_fr(K, '1');
KangarooChannel K2_fl(K, '2');
KangarooChannel K3_rr(K, '3', 129);
KangarooChannel K4_rl(K, '4', 129);

void delay(int milliseconds)
{
    useconds_t microseconds = milliseconds * 1000;
    usleep(microseconds);
}

void setup()
{
  K1_fr.start();
  K2_fl.start();
  K3_rr.start();
  K4_rl.start();
}

void loop()
{

  long minimum = K1_fr.getMin().value();
  long maximum = K1_fr.getMax().value();
  long speed   = (maximum - minimum) / 10; // 1/10th of the range per second

  K1_fr.s(speed);
  delay(1000);
  K1_fr.s(0);
  delay(1000);

  K1_fr.s(-speed);
  delay(1000);
  K1_fr.s(0);
  delay(1000);

  minimum = K2_fl.getMin().value();
  maximum = K2_fl.getMax().value();
  speed   = (maximum - minimum) / 10; // 1/10th of the range per second

  K2_fl.s(speed);
  delay(1000);
  K2_fl.s(0);
  delay(1000);

  K2_fl.s(-speed);
  delay(1000);
  K2_fl.s(0);
  delay(1000);

  minimum = K3_rr.getMin().value();
  maximum = K3_rr.getMax().value();
  speed   = (maximum - minimum) / 10; // 1/10th of the range per second

  K3_rr.s(speed);
  delay(1000);
  K3_rr.s(0);
  delay(1000);

  K3_rr.s(-speed);
  delay(1000);
  K3_rr.s(0);
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
