// #line 1 "G:\\My Drive\\2022 UVU Fall\\Senior Design Project\\Tello Project\\MPU6050 Arduino\\test_mpu.ino"

/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll & Yaw Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() 
{
  Serial.begin(9600);
  // while (true) {
  //   Serial.println("BLAAAA");
  // }
  delay(10000);
  Serial.println("1");
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  Serial.println("2");
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
  
  Serial.println("3");

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
  Serial.println("4");
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  //pitch = pitch + norm.YAxis * timeStep;
  //roll = roll + norm.XAxis * timeStep;
  //yaw = yaw + norm.ZAxis * timeStep;

  delay(3000);
  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(norm.YAxis);
  Serial.print(" Roll = ");
  Serial.print(norm.XAxis);  
  Serial.print(" Yaw = ");
  Serial.println(norm.ZAxis);

  // Wait to full timeStep period
  //delay((timeStep*1000) - (millis() - timer));
}
