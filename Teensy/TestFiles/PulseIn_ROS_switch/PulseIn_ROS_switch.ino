//RC controller must be running
//switch between ROS and RC control
  //uses interrupts with timer library to check switch status
//reads RC signals with PulseIn and converts to velocity or switch signal
#include<Kangaroo.h>
#include<Timer.h>

//Kangaroo Serial channel- Teensy pins 0, 1
#define KSERIAL Serial1

//RC signal pin connections
#define D_PIN 5
#define T_PIN 6
#define RC_SWITCH 10

//Constants
#define D_MAX_RC 1886
#define D_MIN_RC 1080
#define T_MAX_RC 1885
#define T_MIN_RC 1057
#define MAX_VEL 100
#define MIN_VEL -100
#define DEADBAND 10
#define SWITCH_THRESHOLD 1450
#define TIMEOUT 1000 //ms

//global variables for RC signal inputs
int D_pulse;
int T_pulse;
int S_pulse;
int S_begin;

//Velocities for Kangaroo S command from RC signal
long Rvel, Lvel;

KangarooSerial  K(KSERIAL);
KangarooChannel K_fr(K, '1'); // front right wheel
KangarooChannel K_fl(K, '2'); // front left wheel
KangarooChannel K_rr(K, '3', 129); // rear right wheel
KangarooChannel K_rl(K, '4', 129); // rear left wheel

Timer t;

void setup() {
  
  Serial.begin(9600);
  KSERIAL.begin(9600);
  
  pinMode(D_PIN, INPUT);
  pinMode(T_PIN, INPUT);
  pinMode(RC_SWITCH, INPUT);

  S_begin = pulseIn(RC_SWITCH, HIGH);

  if(S_begin > SWITCH_THRESHOLD)
  {
    K_Init();
  }
  
  t.every(1000, checkSwitch, (void*)0);
  
}

//Serial Event interrupt
//Loop should be empty -- timer interrupt to call pulsein
void loop() {
  
  t.update();
  
  if(S_begin > SWITCH_THRESHOLD && S_pulse < SWITCH_THRESHOLD){
    S_begin = S_pulse;
    delay(TIMEOUT); //Serial Timeout
    
  }else if(S_begin < SWITCH_THRESHOLD && S_pulse > SWITCH_THRESHOLD){
    S_begin = S_pulse;
    K_Init();
  }else if(S_pulse > SWITCH_THRESHOLD){
    RC_control();
  }

}

void K_Init()
{
    K_fr.start();
    K_fr.serialTimeout(TIMEOUT);
    K_fr.home().wait();

    K_fl.start();
    K_fl.serialTimeout(TIMEOUT);
    K_fl.home().wait();

    K_rr.start();
    K_rr.serialTimeout(TIMEOUT);
    K_rr.home().wait();

    K_rl.start();
    K_rr.serialTimeout(TIMEOUT);
    K_rl.home().wait();
}


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

void serialEvent(){

  while(Serial.available() && S_begin < SWITCH_THRESHOLD){
    int dataByte;
    dataByte = Serial.read();
    KSERIAL.write(dataByte);
  }
}

void serialEvent1(){

  while(KSERIAL.available() > 0 && S_begin < SWITCH_THRESHOLD){
    int dataByte;
    dataByte = KSERIAL.read();
    Serial.write(dataByte);
  }
}

void checkSwitch(void *context){
  S_pulse = pulseIn(RC_SWITCH, HIGH); 
}

void RC_control(){
    D_pulse = pulseIn(D_PIN, HIGH);
    T_pulse = pulseIn(T_PIN,HIGH);
  
    calcVel();
    Serial.println("R & L velocities");
    Serial.println(Rvel);
    Serial.println(Lvel);
    K_fr.s(Rvel);
    K_rr.s(Rvel);
    K_fl.s(Lvel);
    K_rl.s(Lvel);
}
