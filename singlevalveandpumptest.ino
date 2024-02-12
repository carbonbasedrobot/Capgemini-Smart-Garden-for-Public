int pumpPin = 5;
int valvePin=6;

// valve pins and solenoid number
// one= pin12
// two=pin 11
// ...a
// seven=pin 6


void setup() {
  // put your setup code here, to run once:
  
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, HIGH);
  delay(1000);
  digitalWrite(pumpPin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
