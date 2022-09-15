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

//this is for the Arduino MKR1000 Wifi board and the GT-U7 GPS module

#include <Arduino.h>
TinyGPSPlus gps;

void setup() {
  //Serial1 is pins 13 and 14 on the MKR1000
  Serial1.begin(9600);
  //start COM over USB for debug data
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  //recieve the GPS packet
  while(Serial1.available() > 0)
  {
    //parse the packet
    gps.encode(Serial1.read());
  }

  //print to COM when location and satilite count is updated
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
