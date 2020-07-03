/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 30/06/2020 
*/

// #define DEBUG // Serial DEBUG
#define DEBUG_MOTOR // StepperMotor DEBUG

#define LMS_T 8 //PIN -Top LimitSwitch
#define LMS_B 9 //PIN -Bottom LimitSwitch
// #define EN_PIN 4 //ENA -Enable
#define DIR 4 //DIR -Direction
#define STEP 5 //PUL -Pulse

uint8_t move_motor;

enum MOVE_MOTOR {
  STATE_ACCL,
  STATE_STEADY
};

uint16_t MICROSTEP = 100;
uint16_t PULSE_GEN = 16000;
uint16_t count = 1;
uint16_t hth = 1;
uint16_t i = 1;

int READ_LMS_T = 0;
int READ_LMS_B = 0;
bool val_lmS_T = false;
bool val_lmS_B = false;

/* procedure move motor stepper */
void stpMov(){
  switch (move_motor){
  case STATE_ACCL:
    for(i = 1; i <= PULSE_GEN; i++){
      if(hth >= 640){
        hth = 0;
        MICROSTEP = MICROSTEP - 1;
      }
      #ifdef DEBUG_MOTOR
      digitalWrite(STEP,1); delayMicroseconds(MICROSTEP); 
      digitalWrite(STEP,0); delayMicroseconds(MICROSTEP);
      #endif
      #ifdef DEBUG
      Serial.print(MICROSTEP);Serial.print(" | ");;Serial.print(hth);Serial.print(" | "); Serial.print(i); Serial.print(" | "); Serial.println(count);
      #endif
      if(count > PULSE_GEN){
        move_motor = STATE_STEADY;
        break;
      }
      count ++;hth ++;
    }
    break;
  case STATE_STEADY:
    readPot();
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP,1); delayMicroseconds(MICROSTEP); 
    digitalWrite(STEP,0); delayMicroseconds(MICROSTEP);
    #endif
    break;
  default:
    break;
  }
}
void readPot(){
  int a = analogRead(A0);
  int mH = map(a, 0, 1023, 50, 25);
  MICROSTEP = mH;
  #ifdef DEBUG
  Serial.println(mH);
  #endif
}
void rdLmsT(){
  int read = digitalRead(LMS_T);
  if(read==0){
    val_lmS_T = true;val_lmS_B = false;
    MICROSTEP = 100; PULSE_GEN = 16000; count = 1; i = 1;
  }
}
void rdLmsB(){
  int read = digitalRead(LMS_B);
  if(read==0){
    val_lmS_B = true;val_lmS_T = false;
  }
}


/* Initialize program */
void setup() {
  // initilize interface serial
  Serial.begin(9600);
  Serial.println("START!!!");

  // initilize digital pin 
  pinMode(STEP, OUTPUT); 
  pinMode(DIR, OUTPUT);  
  // pinMode(EN_PIN, OUTPUT);

  pinMode(LMS_T, INPUT_PULLUP);  
  pinMode(LMS_B, INPUT_PULLUP);

  // initial enable and direction motor move up
  // digitalWrite(EN_PIN,0);  
  digitalWrite(DIR,0);
  move_motor = STATE_ACCL;
}

/* Main loop program */
void loop() {
  // #ifdef DEBUG
  // READ_LMS_T = digitalRead(LMS_T); READ_LMS_B = digitalRead(LMS_B);
  // Serial.print("value limit switch: "); Serial.print(READ_LMS_T); Serial.print(" | "); Serial.println(READ_LMS_B);
  // #endif
  rdLmsT();rdLmsB();
  if(val_lmS_T){ 
    digitalWrite(DIR, 0);stpMov();
  } else {
    if(val_lmS_B){ 

    }
  }
}
