void setup(){
  Serial.begin(9600);
  pinMode(7, OUTPUT);
}

void loop(){
  digitalWrite(7, 0);
  Serial.println("KAHFI LOVERS");
  delay(3000);
  // digitalWrite(11, 0);
  // Serial.println("DICKI");
  // delay(3000);
}