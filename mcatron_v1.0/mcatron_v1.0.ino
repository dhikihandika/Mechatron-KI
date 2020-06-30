/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 08/06/2020 
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
#define STEP_P 6 //PIN -Pulse Step 
#define BZZ 8 //PIN -Buzzer
#define LSR 9 //PIN -Lasser

//
#define V_MCRSTP 3000 //MCRSTP delay value
#define V_PGEN 1000 //Pulse generate value
#define V_DEC 2 //Decreament value
#define UP 1 //Move UP stepper motor
#define DOWN 0 //Move Down stepper motor

unsigned long crntMils = 0; //current millis timer 
unsigned long crntStrMils = 0;
unsigned long prevMils = 0; //buffer to previous millis timer
unsigned long prevStpMils = 0; //buffer to previous millis timer

uint16_t MCRSTP = V_MCRSTP; //Microstep declaration 
uint16_t PGEN = V_PGEN; //Pulse generate declaration
uint16_t cnt = 1; //Buffer count
uint16_t i = 1; //BUffer for loop condition

int RD_LMS_T = 0; //read value of top limit switch 
int RD_LMS_B = 0; //read value of bottom limit switch 
bool INIT = false; //initialize value
bool V_LMS_T = false; //Value of top limit switch
bool V_LMS_B = false; //Value of bottom limit switch
bool START = false;
bool STOP = false;
bool DETECTOR = false;

/* procedure move motor stepper */
void stpMov(){
  for(i = 1; i <= PGEN; i++){
    MCRSTP = MCRSTP - V_DEC;
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP_P,HIGH); delayMicroseconds(MCRSTP); // dutyCycle its 50%
    digitalWrite(STEP_P,LOW); delayMicroseconds(MCRSTP);
    #endif
    #ifdef DEBUG
    Serial.print(MCRSTP);Serial.print(" | "); Serial.print(i); Serial.print(" | "); Serial.println(cnt);
    #endif
    if(cnt >= PGEN){
      MCRSTP = (V_PGEN + V_DEC);
      cnt = PGEN;
      break;
    }
    cnt ++;
  }
}

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

  pinMode(STEP_P, OUTPUT); 
  pinMode(DIR, OUTPUT);  
  // pinMode(EN, OUTPUT);
  pinMode(BZZ, OUTPUT);
  pinMode(LSR, OUTPUT);
 
  // interrupt I/O
  attachInterrupt(digitalPinToInterrupt(LMS_T), rdLmsT, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMS_B), rdLmsB, FALLING);
  attachInterrupt(digitalPinToInterrupt(DET), detGo, FALLING);
  attachInterrupt(digitalPinToInterrupt(STR), strRun, LOW);
  attachInterrupt(digitalPinToInterrupt(STP), stpRun, LOW);
}

/* Function use to SoftReset MCU */
void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  #ifdef DEBUG
  RD_LMS_T = digitalRead(LMS_T); RD_LMS_B = digitalRead(LMS_B);
  Serial.print("value limit switch: "); Serial.print(RD_LMS_T); Serial.print(" | "); Serial.println(RD_LMS_B);
  #endif
  if(V_LMS_T && !START){
    crntStrMils = millis();
  } else {
    crntMils = millis();
    if(crntMils - prevMils >= 5010){
      digitalWrite(BZZ, LOW); //Buzzer of
      while (!INIT){
      digitalWrite(DIR, UP);stpMov(); //Motor move up
      }
    } else {
      digitalWrite(BZZ, HIGH);  //Buzzer ON 5000ms
    }
  }

  if(crntStrMils - prevMils >= 5010){
    digitalWrite(BZZ, LOW);
    if(START){
      digitalWrite(DIR, DOWN);stpMov();
      }
    } else {
      if(V_LMS_T){
        digitalWrite(LSR, HIGH);
      }
      digitalWrite(BZZ, HIGH);
  } 

  if(DETECTOR || V_LMS_B){
    if(crntMils - prevStpMils >= 5010){
      digitalWrite(BZZ, LOW);
      if(STOP){
        int a = digitalRead(STP);
        resetFunc();
      }
    } else {
      digitalWrite(LSR, LOW); digitalWrite(BZZ, HIGH);
      START = false; //INIT here or not????
      crntMils = millis();
    }
  } 
}

/* ISR read limit switch */
void rdLmsT(){
  V_LMS_T = true;INIT= true;STOP=false;
  prevMils = millis();
}
void rdLmsB(){
  V_LMS_B = true;
  prevStpMils = millis();
}
void strRun(){
  START = true;
  MCRSTP = V_MCRSTP; PGEN = V_PGEN; cnt = 1; i = 1;
}
void stpRun(){
  STOP = true;
}
void detGo(){
  DETECTOR = true; 
  prevStpMils = millis();
}
