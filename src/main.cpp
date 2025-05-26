#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(38, OUTPUT);
}

void loop() {
  digitalWrite(38, HIGH);  // Turn the pin HIGH
  delay(1000);             // Wait for a second
  digitalWrite(38, LOW);   // Turn the pin LOW
  delay(1000);             // Wait for a second
}
