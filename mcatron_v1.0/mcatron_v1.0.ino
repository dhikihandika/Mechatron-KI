/*
  author  : dhikihandika
  email   : dhikihandika36@gmail.com 
  date    : 08/06/2020 
*/


// #define DEBUG // Serial DEBUG comment

#define LMTSWT_TOP 2 
#define LMTSWT_BOTTOM 3
#define EN_PIN 4 //ENA -Enable
#define DIR_PIN 5  //DIR -Direction
#define STEP_PIN 6 //PUL -Pulse

#define MICROSTEP1 8000
#define MICROSTEP2 7000
#define MICROSTEP3 6000
#define MICROSTEP4 500
#define LOW_MICROSTEP 500
#define timer1 0
#define timer2 5000
#define timer3 10000
#define timer4 20000

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

int READ_LMTSWT_TOP = 0;
int READ_LMTSWT_BOTTOM = 0;
bool val_lmtswt_top = false;
bool val_lmtswt_bottom = false;

/* procedure move motor stepper */
void stpMov1(){
  digitalWrite(STEP_PIN,HIGH); delayMicroseconds(MICROSTEP1); 
  digitalWrite(STEP_PIN,LOW); delayMicroseconds(MICROSTEP1);  
}
void stpMov2(){
  digitalWrite(STEP_PIN,HIGH); delayMicroseconds(MICROSTEP2); 
  digitalWrite(STEP_PIN,LOW); delayMicroseconds(MICROSTEP2);  
}
void stpMov3(){
  digitalWrite(STEP_PIN,HIGH); delayMicroseconds(MICROSTEP3); 
  digitalWrite(STEP_PIN,LOW); delayMicroseconds(MICROSTEP3);  
}
void stpMov4(){
  digitalWrite(STEP_PIN,HIGH); delayMicroseconds(MICROSTEP4); 
  digitalWrite(STEP_PIN,LOW); delayMicroseconds(MICROSTEP4);  
}

void stpMove(){
  currentMillis = millis();
  if((currentMillis - previousMillis >= timer1) && (currentMillis - previousMillis < timer2)){
    stpMov1();
  } else {
    if((currentMillis - previousMillis >= timer2) && (currentMillis - previousMillis < timer3)){
      stpMov2();
    } else {
      if((currentMillis - previousMillis >= timer3) && (currentMillis - previousMillis < timer4)){
        stpMov3();
      } else {
        if(currentMillis - previousMillis >= timer4){
          stpMov4();
        }
      }
    }
  }
}

void setup() {
  // initilize interface serial
  Serial.begin(9600);

  // initilize digital pin 
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT);  
  pinMode(EN_PIN, OUTPUT);

  pinMode(LMTSWT_TOP, INPUT_PULLUP);  
  pinMode(LMTSWT_BOTTOM, INPUT_PULLUP);

  // initial enable and direction motor move up
  digitalWrite(EN_PIN,LOW);  
  digitalWrite(DIR_PIN,LOW);

  // interrupt 
  attachInterrupt(digitalPinToInterrupt(LMTSWT_TOP), readLmtSwt_Top, FALLING);
  attachInterrupt(digitalPinToInterrupt(LMTSWT_BOTTOM), readLmtSwt_Bottom, FALLING);
}

void loop() {
  #ifdef DEBUG
  READ_LMTSWT_TOP = digitalRead(LMTSWT_TOP); READ_LMTSWT_BOTTOM = digitalRead(LMTSWT_BOTTOM);
  Serial.print("value limit switch: "); Serial.print(READ_LMTSWT_TOP); Serial.print(" | "); Serial.println(READ_LMTSWT_BOTTOM);
  #endif

  if(val_lmtswt_top){
    digitalWrite(EN_PIN, LOW); digitalWrite(DIR_PIN, HIGH);
    stpMove();
  } else {
    if(val_lmtswt_bottom){
      digitalWrite(EN_PIN, LOW); digitalWrite(DIR_PIN, LOW);
      stpMove();
    }
  }
}

/* ISR read limit switch */
void readLmtSwt_Top(){
  val_lmtswt_top = true;val_lmtswt_bottom = false;
  previousMillis = millis();
}
void readLmtSwt_Bottom(){
  val_lmtswt_bottom = true;val_lmtswt_top = false;
  previousMillis = millis();
}
