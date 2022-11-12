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

  // Countdown 10 seconds to allow MPU to warm up
  for (int i = 10; i > 0; i--)
  {
    Serial.println(i);
    delay(1000);
  }
  
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
    
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

 // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
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

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(norm.YAxis);
  Serial.print("\t\tRoll = ");
  Serial.print(norm.XAxis);  
  Serial.print("\t\tYaw = ");
  Serial.println(norm.ZAxis);

  //use this to change how often output updates
  delay(500);

  // Wait to full timeStep period
  //delay((timeStep*1000) - (millis() - timer));
}
