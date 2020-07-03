/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 02/07/2020 
*/

// #define DEBUG // Serial DEBUG
#define DEBUG_MOTOR // StepperMotor DEBUG

#define STR 2 //PIN -PushButton Start
#define STP 3 //PIN -PushButton Stop
#define LMS_T 8 //PIN -Top LimitSwitch
#define LMS_B 9 //PIN -Bottom LimitSwitch
#define DET 10 //PIN - Detector (IR Sensor)
#define OPR 11 //PIN - LED indicator operation


// #define EN 4 //PIN -Enable   
#define DIR 4 //PIN -Direction
#define STEP 5 //PIN -Pulse Step 
#define BZZ 6 //PIN -Buzzer
#define LSR 7 //PIN -Lasser
#define UP 1 //Move UP stepper motor
#define DOWN 0 //Move Down stepper motor

enum MOVE_MOTOR {
  STATE_ACCL,
  STATE_STEADY
};

enum STATE_PROCESS{
  STATE_INIT,
  STATE_PLACE_IN_UP,
  STATE_ADJUST_POSITION,
  STATE_AFTER_ADJUST
};

unsigned long crntMils=0;unsigned long crntMils1=0;unsigned long crntMils2=0;
unsigned long prevMils=0;unsigned long prevMils1=0;unsigned long prevMils2=0;
uint8_t move_motor=STATE_ACCL;
uint8_t state_process=STATE_INIT; //initial procces system

uint16_t MCRSTP=100; //Microstep declaration 
uint16_t PGEN=16000; //Pulse generate declaration
uint16_t cnt=1;uint16_t hth=1; //Buffer count
uint16_t i=1; //BUffer for loop condition

uint16_t clock=0; 
uint16_t clock1=0;bool tick1=false;
uint16_t clock2=0;bool tick2=false;


bool pbStp=false;int pbStr=0;
bool lmsTop=false;bool lmsBot=false;
bool dtctr=false;

//==============================//
/* procedure move motor stepper */
//==============================//
void stpMov(){
  for(i=1;i<=PGEN;i++){
    if(hth>=320){
      hth=0;
      MCRSTP=MCRSTP-1;
    }
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP,1); delayMicroseconds(MCRSTP); // dutyCycle its 50%
    digitalWrite(STEP,0); delayMicroseconds(MCRSTP);
    #endif
    #ifdef DEBUG
    Serial.print(MCRSTP);Serial.print(" | ");Serial.print(i);Serial.print(" | ");Serial.println(cnt);
    #endif
    if(cnt>=PGEN){
      MCRSTP=50;
      // readPot();
      cnt=PGEN;
      break;
    }
    cnt++;hth++;
  }
}

//====================================//
/* procedure read value DI and AI */
//====================================//
void readPot(){
  int a=analogRead(A0);
  int mH=map(a,0,1023,50,25);
  MCRSTP=mH;
}
void rdLmsT(){
  int read=digitalRead(LMS_T);
   if(read==0){
    lmsTop=true;
  }
}
void rdLmsB(){
  int read=digitalRead(LMS_B);
  if(read==0){
    if(state_process==STATE_ADJUST_POSITION){
    lmsBot=true;
    }
  }
}
void detGo(){
  int read=digitalRead(DET);
  if(read==0){
    if(state_process==STATE_ADJUST_POSITION){
    dtctr=true;
    }
  }
}

//========================//
/* procedure millis clock */
//========================//
void millisClock(){
  crntMils=millis();
  if((crntMils-prevMils>=1000)&&(crntMils-prevMils<=1005)){
    clock++;
    prevMils=millis();
  }
}
void millisClock1(){
  crntMils1=millis();
  if((crntMils1-prevMils1>=1000)&&(crntMils1-prevMils1<=1005)){
    clock1++;
    prevMils1=millis();
  }
}
void millisClock2(){
  crntMils2=millis();
  if((crntMils2-prevMils2>=1000)&&(crntMils2-prevMils2<=1005)){
    clock2++;
    prevMils2=millis();
  }
}

//=======================//
/* Initilize the program */
//=======================//
void setup(){
  // initilize interface serial
  Serial.begin(9600);
  Serial.println("START!!!");

  // initilize digital I/O pin 
  pinMode(LMS_T,INPUT_PULLUP);  
  pinMode(LMS_B,INPUT_PULLUP);
  pinMode(STR,INPUT_PULLUP);
  pinMode(STP,INPUT_PULLUP);
  pinMode(DET,INPUT_PULLUP);

  pinMode(STEP,OUTPUT); 
  pinMode(DIR,OUTPUT);  
  // pinMode(EN, OUTPUT);
  pinMode(BZZ,OUTPUT);
  pinMode(LSR,OUTPUT);
  pinMode(OPR,OUTPUT);
  
  // setup state proccess
  move_motor=STATE_ACCL; 
  state_process=STATE_INIT;

  // interrupt I/O
  attachInterrupt(digitalPinToInterrupt(STR),strRun,LOW);
  attachInterrupt(digitalPinToInterrupt(STP),stpRun,LOW);
}

//=======================//
/* main loop program */
//=======================//
void loop() {
  millisClock();
// put your main code here, to run repeatedly:
  switch(state_process){
  case STATE_INIT:
    lmsBot=pbStr=pbStp=dtctr=false;//Disable all variable can't use
    if(clock>=5){
      rdLmsT();
      digitalWrite(BZZ,0);
      digitalWrite(OPR,1);digitalWrite(DIR,UP);stpMov();
      if(lmsTop){
        digitalWrite(OPR,0);
        state_process=STATE_PLACE_IN_UP;
        clock1=0;prevMils1=millis();
      }
    } else {
      MCRSTP=100;PGEN=16000;cnt=1;i=1;
      digitalWrite(BZZ,1);
    }
    break;
  case STATE_PLACE_IN_UP:
    lmsTop=pbStp=false;//Disabla all variable can't use
    millisClock1();
    if(clock1>=5){
      if(pbStr){
        state_process=STATE_ADJUST_POSITION;
        lmsBot=dtctr=false;
      } else {
        digitalWrite(BZZ,0);
      }
    } else {
      digitalWrite(BZZ,1);digitalWrite(LSR,1);
    }
    break;
  case STATE_ADJUST_POSITION:
    lmsTop=pbStr=pbStp=false;//Disable all variable can't use
    rdLmsB();detGo();
    if(lmsBot||dtctr){
      digitalWrite(OPR,0);
      state_process=STATE_AFTER_ADJUST;prevMils2=millis();clock2=0;
    } else {
      digitalWrite(OPR,1);digitalWrite(DIR,DOWN);stpMov();
    }
    break;
  case STATE_AFTER_ADJUST:
    lmsTop=lmsBot=pbStr=dtctr=false;
    millisClock2();
    if(clock2>=5){
      digitalWrite(BZZ,0);
      if(pbStp){
        lmsTop=lmsBot=pbStp=pbStp=dtctr=false;
        state_process=STATE_INIT;
        clock=0;prevMils=millis();
      }
    } else {
      digitalWrite(BZZ,1);digitalWrite(LSR,0);
    }
    break;
  default:
    break;
  }
}

//=======================//
/* ISR read limit switch */
//=======================//
void strRun(){
  pbStr=true;
  MCRSTP=100;PGEN=16000;cnt=1;i=1;
}
void stpRun(){
  pbStp=true;
}
