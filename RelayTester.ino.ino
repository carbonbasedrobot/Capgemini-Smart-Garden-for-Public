void setup() {
  // put your setup code here, to run once:
  for(int i = 5; i<12; i++){
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 6; i<13; i++){

      digitalWrite(i, HIGH);
      delay(2000);
      digitalWrite(5, HIGH);
      delay(2000);
      digitalWrite(5, LOW);
      delay(2000);
      digitalWrite(i, LOW);
      delay(2000);
  }

  //  digitalWrite(6, HIGH);
  //  delay(1000);
  //  digitalWrite(5, HIGH);
  //  delay(1000);
  //  digitalWrite(5, LOW);
  //  delay(1000);
  //  digitalWrite(6, LOW);
  //  delay(1000);
}
