void setup(){
  Serial.begin(9600);
  pinMode(11, OUTPUT);
}

void loop(){
  digitalWrite(11, 0);
  delay(5000);
  digitalWrite(11, 1);
  delay(3000);
}