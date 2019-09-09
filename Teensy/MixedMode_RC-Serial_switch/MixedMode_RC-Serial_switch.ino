//RC transmitter must be turned on prior to providing power to receiver

//Must be tuned in mixed mode
//Channel names:
// Addr 128 - Drive = 'D' and Turn = 'T'
// Addr 129 - Drive = 'L' and Turn = 'A'

#include<Kangaroo.h>
#include<Timer.h>

//Kangaroo Serial channel- Teensy pins 0, 1
#define KSERIAL Serial1

//RC signal pin connections
#define D_PIN 5 //receiver channel 2
#define T_PIN 6 //receiver channel 4
#define RC_SWITCH 10 //receiver channel 5

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
KangarooChannel K_fd(K, 'D'); // front drive
KangarooChannel K_ft(K, 'T'); // front drive
KangarooChannel K_rd(K, 'L', 129); // rear drive
KangarooChannel K_rt(K, 'A', 129); // rear drive

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
    K_fd.start();
    K_fd.serialTimeout(TIMEOUT);
    K_fd.home().wait();

    K_ft.start();
    K_ft.serialTimeout(TIMEOUT);
    K_ft.home().wait();

    K_rd.start();
    K_rd.serialTimeout(TIMEOUT);
    K_rd.home().wait();

    K_rt.start();
    K_rt.serialTimeout(TIMEOUT);
    K_rt.home().wait();
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

    long D_val = map(D_pulse, D_MIN_RC, D_MAX_RC, MIN_VEL, MAX_VEL);
    long T_val = map(T_pulse, T_MIN_RC, T_MAX_RC, MIN_VEL, MAX_VEL);

    Serial.println("Drive, Turn");
    Serial.println(D_val);
    Serial.println(T_val);
    if(-DEADBAND < D_val && D_val < DEADBAND)
    {
      D_val = 0;
    }
    if(-DEADBAND < T_val && T_val < DEADBAND)
    {
      T_val = 0;
    }
    KangarooStatus sfd = K_fd.getS();
    KangarooStatus sft = K_rt.getS();
    KangarooStatus srd = K_fd.getS();
    KangarooStatus srt = K_rt.getS();

    bool statusOK = sfd.ok() && sft.ok() && srd.ok() && srt.ok();
    if(!statusOK)
    {
      //Serial.println("ERROR");
      Serial.println("ERROR Status d, t");
      Serial.println(sfd.error());
      Serial.println(sft.error());
      Serial.println(srd.error());
      Serial.println(srt.error());

      delay(1000);

      //if control error, error will be cleared by issuing start command

      K_fd.start();
      K_ft.start();
      K_rd.start();
      K_rt.start();
      
    }else{
      K_fd.s(D_val).wait();
      K_ft.s(T_val).wait();
      K_rd.s(D_val).wait();
      K_rt.s(T_val).wait();
    }
}
