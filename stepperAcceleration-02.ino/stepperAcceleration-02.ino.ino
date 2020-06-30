/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 30/06/2020 
*/

// #define DEBUG // Serial DEBUG
#define DEBUG_MOTOR // StepperMotor DEBUG

#define LMS_T 2 //PIN -Top LimitSwitch
#define LMS_B 3 //PIN -Bottom LimitSwitch
// #define EN_PIN 4 //ENA -Enable
#define DIR 5 //DIR -Direction
#define STEP 6 //PUL -Pulse

uint16_t MICROSTEP_H = 380;
uint16_t MICROSTEP_L = 25;
uint16_t PULSE_GEN = 16000;
uint16_t count = 1;
uint16_t htt = 1;
uint16_t hth = 1;
uint16_t i = 1;

int READ_LMS_T = 0;
int READ_LMS_B = 0;
bool val_lmS_T = false;
bool val_lmS_B = false;

/* procedure move motor stepper */
void stpMov(){
  for(i = 1; i <= PULSE_GEN; i++){
    if(htt == 1600){
      htt = 0;
      MICROSTEP_L = MICROSTEP_L - 1;  
    }
    if(hth == 100){
      hth = 0;
      MICROSTEP_H = MICROSTEP_H - 1;
    }
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP,1); delayMicroseconds(MICROSTEP_H); 
    digitalWrite(STEP,0); delayMicroseconds(MICROSTEP_L);
    #endif
    #ifdef DEBUG
    Serial.print(MICROSTEP_H);Serial.print(" | ");Serial.print(MICROSTEP_L);Serial.print(" | ");Serial.print(htt);Serial.print(" | ");Serial.print(hth);Serial.print(" | "); Serial.print(i); Serial.print(" | "); Serial.println(count);
    #endif
    if(count > PULSE_GEN){
      MICROSTEP_H = 220;
      MICROSTEP_L = 15;
      count = PULSE_GEN;
      readPot();
      break;
    }
    count ++;
    htt ++; hth ++;
  }
}

void readPot(){
  int a = analogRead(A0);
  int mL = map(a, 0, 1023, 15, 10);
  int mH = map(a, 0, 1023, 220, 140);
  MICROSTEP_H = mH;
  MICROSTEP_L = mL;
  #ifdef DEBUG
  Serial.print(mH);Serial.print("|");Serial.println(mL);
  #endif
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

  // interrupt I/O
  attachInterrupt(digitalPinToInterrupt(LMS_T), readLmS_T, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMS_B), readLmS_B, FALLING);
}

/* Main loop program */
void loop() {
  // #ifdef DEBUG
  // READ_LMS_T = digitalRead(LMS_T); READ_LMS_B = digitalRead(LMS_B);
  // Serial.print("value limit switch: "); Serial.print(READ_LMS_T); Serial.print(" | "); Serial.println(READ_LMS_B);
  // #endif
  if(val_lmS_T){ 
    digitalWrite(DIR, 0);stpMov();
  } else {
    if(val_lmS_B){ 

    }
  }
}

/* ISR (Interrupt Service Routine) read limit switch */
void readLmS_T(){
  val_lmS_T = true;val_lmS_B = false;
  MICROSTEP_H = 380;MICROSTEP_L = 25; PULSE_GEN = 16000; count = 1; i = 1;
}
void readLmS_B(){
  val_lmS_B = true;val_lmS_T = false;
}
