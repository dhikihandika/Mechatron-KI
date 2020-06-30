/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 17/06/2020 
*/

// #define DEBUG // Serial DEBUG
#define DEBUG_MOTOR // StepperMotor DEBUG

#define LMS_T 2 //PIN -Top LimitSwitch
#define LMS_B 3 //PIN -Bottom LimitSwitch
#define STR 18 //PIN -PushButton Start
#define STP 19 //PIN -PushButton Stop
#define DET 20 //PIN - Detector (IR Sensor)

// #define EN 4 //PIN -Enable
#define DIR 5 //PIN -Direction
#define STEP 6 //PIN -Pulse Step 
#define BZZ 8 //PIN -Buzzer
#define LSR 9 //PIN -Lasser
#define UP 1 //Move UP stepper motor
#define DOWN 0 //Move Down stepper motor

enum STATE_PROCESS{
  STATE_INIT,
  STATE_PLACE_IN_UP,
  STATE_ADJUST_POSITION,
  STATE_AFTER_ADJUST
};

unsigned long crntMils = 0;unsigned long crntMils1 = 0;unsigned long crntMils2 = 0;
unsigned long prevMils = 0;unsigned long prevMils1 = 0;unsigned long prevMils2 = 0;
uint8_t state_process = STATE_INIT; //initial procces system

uint16_t MCRSTP = 3000; //Microstep declaration 
uint16_t PGEN = 1000; //Pulse generate declaration
uint16_t cnt = 1; //Buffer count
uint16_t i = 1; //BUffer for loop condition
uint16_t clock = 0; 
uint16_t clock1 = 0;bool tick1 = false;
uint16_t clock2 = 0;bool tick2 = false;


bool initilize = false;
bool pbStp = false;int pbStr = 0;
bool lmsTop = false;bool lmsBot = false;
bool dtctr = false;

/* procedure move motor stepper */
void stpMov(){
  for(i = 1; i <= PGEN; i++){
    MCRSTP = MCRSTP - 2;
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP,1); delayMicroseconds(MCRSTP); // dutyCycle its 50%
    digitalWrite(STEP,0); delayMicroseconds(MCRSTP);
    #endif
    #ifdef DEBUG
    Serial.print(MCRSTP);Serial.print(" | "); Serial.print(i); Serial.print(" | "); Serial.println(cnt);
    #endif
    if(cnt >= PGEN){
      MCRSTP = (1000 + 2);
      cnt = PGEN;
      break;
    }
    cnt ++;
  }
}

void millisClock(){
  crntMils = millis();
  if((crntMils - prevMils >= 1000)&&(crntMils - prevMils <= 1005)){
    clock++;
    prevMils = millis();
  }
}

void millisClock1(){
  crntMils1 = millis();
  if((crntMils1 - prevMils1 >= 1000)&&(crntMils1 - prevMils1 <= 1005)){
    clock1++;
    prevMils1 = millis();
  }
}

void millisClock2(){
  crntMils2 = millis();
  if((crntMils2 - prevMils2 >= 1000)&&(crntMils2 - prevMils2 <= 1005)){
    clock2++;
    prevMils2 = millis();
  }
}

/* initilize the program */
void setup() {
  // initilize interface serial
  Serial.begin(9600);
  Serial.println("START!!!");

  // initilize digital I/O pin 
  pinMode(LMS_T, INPUT_PULLUP);  
  pinMode(LMS_B, INPUT_PULLUP);
  pinMode(STR, INPUT_PULLUP);
  pinMode(STP, INPUT_PULLUP);
  pinMode(DET, INPUT_PULLUP);

  pinMode(STEP, OUTPUT); 
  pinMode(DIR, OUTPUT);  
  // pinMode(EN, OUTPUT);
  pinMode(BZZ, OUTPUT);
  pinMode(LSR, OUTPUT);
 
  // setup state proccess Ready
  state_process = STATE_INIT;

  // interrupt I/O
  attachInterrupt(digitalPinToInterrupt(LMS_T), rdLmsT, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMS_B), rdLmsB, FALLING);
  attachInterrupt(digitalPinToInterrupt(DET), detGo, FALLING);
  attachInterrupt(digitalPinToInterrupt(STR), strRun, LOW);
  attachInterrupt(digitalPinToInterrupt(STP), stpRun, LOW);
}

/* Function use to SoftReset MCU */
void(* resetFunc) (void) = 0; //declare reset function @ address 0

/* main loop program */
void loop() {
  millisClock();
// put your main code here, to run repeatedly:
  switch (state_process){
  case STATE_INIT:
    lmsBot=false;pbStr=false;pbStp=false;dtctr=false;//Disable all variable can't use
    if(clock >= 5){
      digitalWrite(BZZ, 0);
      digitalWrite(DIR, UP); stpMov();
      if(lmsTop){
        state_process = STATE_PLACE_IN_UP;prevMils1 = millis();
      }
    } else {
      digitalWrite(BZZ, 1);
    }
    break;
  case STATE_PLACE_IN_UP:
    lmsTop=false;pbStp=false;//Disabla all variable can't use
    millisClock1();
    if(clock1 >= 5){
      if(pbStr){
        state_process = STATE_ADJUST_POSITION;lmsBot=false;dtctr=false;
      } else {
        digitalWrite(BZZ, 0);
      }
    } else {
      digitalWrite(BZZ, 1);digitalWrite(LSR, 1);
    }
    break;
  case STATE_ADJUST_POSITION:
    lmsTop=false;pbStr=false;pbStp=false;//Disable all variable can't use
    if(lmsBot || dtctr){
      state_process = STATE_AFTER_ADJUST; prevMils2 = millis();
    } else {
      digitalWrite(DIR, DOWN);stpMov();
    }
    break;
  case STATE_AFTER_ADJUST:
    lmsTop=false;lmsBot=false;pbStr=false;dtctr=false;
    millisClock2();
    if(clock2 >= 5){
      digitalWrite(BZZ, 0);
      if(pbStp){
        resetFunc();
      }
    } else {
      digitalWrite(BZZ, 1);digitalWrite(LSR, 0);
    }
    break;
  default:
    break;
  }
}

/* ISR read limit switch */
void rdLmsT(){
  lmsTop = true;
}
void rdLmsB(){
  if(state_process == STATE_ADJUST_POSITION){
    lmsBot = true;
  }
}
void strRun(){
  pbStr = true;
  MCRSTP = 3000; PGEN = 1000; cnt = 1; i = 1;
}
void stpRun(){
  pbStp = true;
}
void detGo(){
  if(state_process == STATE_ADJUST_POSITION){
    dtctr = true;
  }
}