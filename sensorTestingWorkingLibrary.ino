#include <HCSR04.h>
UltraSonicDistanceSensor distanceSensor(3, 2);  // Initialize sensor that uses digital pins 13 and 12.

int sensorReading;
int mappedValue;
int tankSensorHeight = 11;

// int readTankHeight() {
//   return map(tankSensorHeight - distanceSensor.measureDistanceCm(), 0, 8, 0, 100);
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorReading = distanceSensor.measureDistanceCm();
  mappedValue = map(tankSensorHeight - sensorReading, 0, tankSensorHeight, 0, 100);
  //mappedValue = tankSensorHeight - sensorReading;
  Serial.println(String(sensorReading));
  delay(50);
}
