//verify conversion of pulseIn input to velocity command
#define D_PIN 5
#define T_PIN 6

#define D_MAX_RC 1886
#define D_MIN_RC 1080
#define T_MAX_RC 1885
#define T_MIN_RC 1057
#define MAX_VEL 500
#define MIN_VEL -500
#define DEADBAND 10

int D_pulse;
int T_pulse;

long Rvel, Lvel;

void setup() {
  
  Serial.begin(9600);
  
  pinMode(D_PIN, INPUT);
  pinMode(T_PIN, INPUT);
  
}

void loop() {
  D_pulse = pulseIn(D_PIN, HIGH);
  T_pulse = pulseIn(T_PIN,HIGH);

  calcVel();

//  Serial.print("Ch 3: ");
//  Serial.println(D_pulse);
//
//  Serial.print("CH 1: ");
//  Serial.println(T_pulse);

}

//long convertPulse(int chx)
//{
//  return map(chx, MIN_RC, MAX_RC, -100, 100);
//}

long VelOutput(long input)
{
  return map(input, -100, 100, MIN_VEL, MAX_VEL);
}

void calcVel()
{
  long D_val, T_val, V, W, R, L;
  
  D_val = map(D_pulse, D_MIN_RC, D_MAX_RC, -100, 100);
  T_val = map(T_pulse, T_MIN_RC, T_MAX_RC, -100, 100);

  T_val = -T_val; //invert

  V = (100-abs(T_val))*(D_val/100.0)+D_val;
  W = (100-abs(D_val))*(T_val/100.0)+T_val;

  R = (V+W)/2;
  if(-DEADBAND < R && DEADBAND > R)
  {
    R = 0;
  }
  Serial.print("Right: ");
  Serial.println(R);
  L = (V-W)/2;
  if(-DEADBAND < L && DEADBAND > L)
  {
    L = 0;
  }
  Serial.print("Left: ");
  Serial.println(L);

  Rvel = VelOutput(R);
  Lvel = VelOutput(L);
  
}
