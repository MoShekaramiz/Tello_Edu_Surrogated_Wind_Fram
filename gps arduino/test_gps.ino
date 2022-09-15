#include <TinyGPSPlus.h>
#include <TinyGPS++.h>
//to get TinyGPS to work, I needed to comment out WProgram.h and change some of the int references
//typedef __int8 int8_t;
//typedef unsigned __int8 uint8_t;
//typedef __int16 int16_t;
//typedef unsigned __int16 uint16_t;
//typedef __int32 int32_t;
//typedef unsigned __int32 uint32_t;

//apparently the library uses a newer version of c++, which is not super compatible with our board

#include <Arduino.h>

#include <Wire.h>
#include <wiring_private.h>

//SERCOM sercom5(sercom5);
//TwoWire myWire(&sercom5, 0, 1);
TinyGPSPlus gps;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(9600);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  while(Serial1.available() > 0)
  {
    //gps.encode(Serial1.read());
   gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated())
  {
    Serial.print("Lat: "); Serial.println(gps.location.lat(), 9);
    Serial.print("Long: "); Serial.println(gps.location.lng(), 9);
  }
    
  if (gps.satellites.isUpdated())
  {
    Serial.print("Sat: "); Serial.println(gps.satellites.value());
  }  
}
