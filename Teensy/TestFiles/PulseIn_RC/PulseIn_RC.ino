//read ppm signals with pulseIn function
#define T_CH3 5
#define A_CH1 6

int ch3;
int ch1;

void setup() {
  
  Serial.begin(9600);
  
  pinMode(T_CH3, INPUT);
  pinMode(A_CH1, INPUT);
  
}

void loop() {
  ch3 = pulseIn(T_CH3, HIGH);
  ch1 = pulseIn(A_CH1,HIGH);

  Serial.print("Ch 3: ");
  Serial.println(ch3);

  Serial.print("CH 1: ");
  Serial.println(ch1);

}
