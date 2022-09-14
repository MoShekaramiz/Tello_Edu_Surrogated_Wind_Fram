#include <TinyGPSPlus.h>
#include <TinyGPS++.h>

#include <Arduino.h>

#include <Wire.h>
#include <wiring_private.h>

//SERCOM sercom5(sercom5);
//TwoWire myWire(&sercom5, 0, 1);

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  TinyGPSPlus gps;
  //myWire.begin();

}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(5000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(5000);                       // wait for a second
}
