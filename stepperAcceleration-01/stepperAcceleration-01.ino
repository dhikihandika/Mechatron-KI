/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 08/06/2020 
*/


// #define DEBUG // Serial DEBUG
#define DEBUG_MOTOR // StepperMotor DEBUG

#define LMTSWT_TOP 2 //PIN -Top LimitSwitch
#define LMTSWT_BOTTOM 3 //PIN -Bottom LimitSwitch
// #define EN_PIN 4 //ENA -Enable
#define DIR_PIN 5 //DIR -Direction
#define STEP_PIN 6 //PUL -Pulse

#define VAL_MICROSTEP 3000
#define VAL_PULSE_GEN 1000
#define VAL_DECREAMENT 2

uint16_t MICROSTEP = VAL_MICROSTEP;
uint16_t PULSE_GEN = VAL_PULSE_GEN;
uint16_t count = 1;
uint16_t i = 1;

int READ_LMTSWT_TOP = 0;
int READ_LMTSWT_BOTTOM = 0;
bool val_lmtswt_top = false;
bool val_lmtswt_bottom = false;

/* procedure move motor stepper */
void stpMov(){
  for(i = 1; i <= PULSE_GEN; i++){
    MICROSTEP = MICROSTEP - VAL_DECREAMENT;
    #ifdef DEBUG_MOTOR
    digitalWrite(STEP_PIN,HIGH); delayMicroseconds(MICROSTEP); 
    digitalWrite(STEP_PIN,LOW); delayMicroseconds(MICROSTEP);
    #endif
    #ifdef DEBUG
    Serial.print(MICROSTEP);Serial.print(" | "); Serial.print(i); Serial.print(" | "); Serial.println(count);
    #endif
    if(count > PULSE_GEN){
      MICROSTEP = (VAL_PULSE_GEN + VAL_DECREAMENT);
      count = PULSE_GEN;
      break;
    }
    count ++;
  }
}

/* Initialize program */
void setup() {
  // initilize interface serial
  Serial.begin(9600);
  Serial.println("START!!!");

  // initilize digital pin 
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);  
  // pinMode(EN_PIN, OUTPUT);

  pinMode(LMTSWT_TOP, INPUT_PULLUP);  
  pinMode(LMTSWT_BOTTOM, INPUT_PULLUP);

  // initial enable and direction motor move up
  // digitalWrite(EN_PIN,LOW);  
  digitalWrite(DIR_PIN,LOW);

  // interrupt I/O
  attachInterrupt(digitalPinToInterrupt(LMTSWT_TOP), readLmtSwt_Top, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMTSWT_BOTTOM), readLmtSwt_Bottom, FALLING);
}

/* Main loop program */
void loop() {
  #ifdef DEBUG
  READ_LMTSWT_TOP = digitalRead(LMTSWT_TOP); READ_LMTSWT_BOTTOM = digitalRead(LMTSWT_BOTTOM);
  Serial.print("value limit switch: "); Serial.print(READ_LMTSWT_TOP); Serial.print(" | "); Serial.println(READ_LMTSWT_BOTTOM);
  #endif
  if(val_lmtswt_top){ 
    stpMov();
  } else {
    if(val_lmtswt_bottom){ 

    }
  }
}

/* ISR (Interrupt Service Routine) read limit switch */
void readLmtSwt_Top(){
  val_lmtswt_top = true;val_lmtswt_bottom = false;
  digitalWrite(DIR_PIN, LOW);
  MICROSTEP = VAL_MICROSTEP; PULSE_GEN = VAL_PULSE_GEN; count = 1; i = 1;
}
void readLmtSwt_Bottom(){
  val_lmtswt_bottom = true;val_lmtswt_top = false;
}
