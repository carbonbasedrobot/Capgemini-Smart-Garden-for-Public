#include <WiFiNINA.h>
// Testing of Analog Pins without Sensors 1/23
// Discovery: A4-A7 have a shorter voltage range of 0-2.7V
//Issue is resolved by updating the wifi nina firmware

// Moisture Sensor Testing 1/17
// Pins: Wet   Dry
// A0:   120   694
// A1:   not working
// A2:   400   880
// A3:   not working
// A4:   not working
// A5:   not working
// A6:   450   1023
// Solution: improve/redo the wiring for the moisture sensors

//

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("A0: " + String(analogRead(A0)));
    Serial.println("A1: " + String(analogRead(A1)));
    Serial.println("A2: " + String(analogRead(A2)));
    Serial.println("A3: " + String(analogRead(A3)));
    Serial.println("A4: " + String(analogRead(A4)));
    Serial.println("A5: " + String(analogRead(A5)));
    Serial.println("A6: " + String(analogRead(A6)));
     Serial.println("A7: " + String(analogRead(A7)));
    Serial.println();
    delay(500);

}
