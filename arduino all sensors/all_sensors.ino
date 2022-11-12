#include <WiFi101.h>
#include <WiFiClient.h>
#include <WiFiMDNSResponder.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiUdp.h>

#include <TinyGPSPlus.h>
#include <TinyGPS++.h>

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>

#include <MPU6050.h>

#include <Arduino.h>

#include <SPI.h>
#include <WiFi101.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
MPU6050 mpu;

char ssid[] = "TELLO-F24367";     // the name of your network
int status = WL_IDLE_STATUS;     // the Wifi radio's status

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
