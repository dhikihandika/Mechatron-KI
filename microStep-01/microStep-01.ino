// defines pins numbers
const int stepPin = 6; 
const int dirPin = 5; 
const int enPin = 4;

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);

  digitalWrite(enPin,LOW);
}

void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 400 pulses for making one full cycle rotation
  for(int x = 0; x < 16000; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(25); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(25); 
  }
  delay(1000); // One second delay

  digitalWrite(dirPin,LOW); // Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 16000; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(25);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(25);
  }
  delay(1000);
}