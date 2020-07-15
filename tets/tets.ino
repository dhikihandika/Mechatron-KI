
#define STR 10 //PIN -PushButton Start
#define STP 11 //PIN -PushButton Stop
#define LMS_T 8 //PIN -Top LimitSwitch
#define LMS_B 9 //PIN -Bottom LimitSwitch
#define DET 2 //PIN - Detector (IR Sensor) ~ optional use if need
#define OPR 6 //PIN - LED indicator operation
#define LSR 7 //PIN -Lasser
#define BZZ 12 //PIN -Buzzer

void setup(){
  Serial.begin(9600);
  pinMode(STR, INPUT_PULLUP);
  pinMode(STP, INPUT_PULLUP);
  pinMode(LMS_T, INPUT_PULLUP);
  pinMode(LMS_B, INPUT_PULLUP);
  pinMode(OPR, OUTPUT);
  pinMode(BZZ, OUTPUT);
  pinMode(LSR, OUTPUT);
}

void loop(){
  int a = digitalRead(STR);
  int b = digitalRead(STP);
  int c = digitalRead(LMS_T);
  int d = digitalRead(LMS_B);

  digitalWrite(OPR, 0);digitalWrite(LSR, 0);digitalWrite(BZZ, 0);
  Serial.print("Str:");Serial.print(a);Serial.print(" |Stp:");Serial.print(b);Serial.print(" |LMS_T:");Serial.print(c);Serial.print(" |LMS_B:");Serial.println(d);
  delay(100);
}