//Move joystick to min and max positions (up, right, down, and left)
//Prints min and max values from RC receiver to serial terminal
#define D_PIN 5 //receiver channel 2
#define T_PIN 6 //receiver channel 4

int D, D_min, D_max;
int T, T_min, T_max;

void setup() {
  
  Serial.begin(9600);
  
  pinMode(D_PIN, INPUT);
  pinMode(T_PIN, INPUT);

  D_min = 2000;
  D_max = 0;

  T_min = 2000;
  T_max = 0;
  
}

void loop() {
  D = pulseIn(D_PIN, HIGH);
  if(D < D_min)
  {
    D_min = D;
  }
  if(D > D_max)
  {
    D_max = D;
  }
  
  T = pulseIn(T_PIN,HIGH);

  if(T < T_min)
  {
    T_min = T;
  }
  if(T > T_max)
  {
    T_max = T;
  }

  Serial.print("D max: ");
  Serial.println(D_max);

  Serial.print("T max: ");
  Serial.println(T_max);
  

  Serial.print("D min: ");
  Serial.println(D_min);

  Serial.print("T min: ");
  Serial.println(T_min);
}
